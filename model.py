import pygame
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Set
import pygame.gfxdraw
import colorsys
from collections import deque

@dataclass
class Hub:
    position: Tuple[int, int]
    spawn_rate: float = 1000  # cars per second
    color: Tuple[int, int, int] = (50, 50, 50)
    size: int = 40
    connected_roads: List[int] = None  # indices of connected roads
    
    def __post_init__(self):
        if self.connected_roads is None:
            self.connected_roads = []

@dataclass
class Road:
    points: List[Tuple[int, int]]
    width: int = 20
    max_speed: int = 5

@dataclass
class Vehicle:
    position: float
    speed: int
    road_index: int
    color: Tuple[int, int, int]
    length: int = 25
    width: int = 16

class Stats:
    def __init__(self, history_length=300):  # 5 seconds at 60fps
        self.history_length = history_length
        self.speed_history = deque(maxlen=history_length)
        self.car_count_history = deque(maxlen=history_length)
        
    def update(self, vehicles):
        if not vehicles:
            avg_speed = 0
        else:
            avg_speed = sum(v.speed for v in vehicles) / len(vehicles)
        self.speed_history.append(avg_speed)
        self.car_count_history.append(len(vehicles))

class ModernUI:
    def __init__(self, screen_width, screen_height):
        self.panel_rect = pygame.Rect(screen_width - 300, 0, 300, screen_height)
        self.font = pygame.font.Font(None, 24)
        self.background_color = (240, 240, 240)
        self.text_color = (50, 50, 50)
        self.graph_color = (100, 100, 200)
        
    def draw_slider(self, screen, pos, value, min_val, max_val, label):
        # Draw slider background
        slider_rect = pygame.Rect(pos[0], pos[1], 200, 10)
        pygame.draw.rect(screen, (200, 200, 200), slider_rect)
        
        # Draw slider handle
        handle_pos = pos[0] + (value - min_val) / (max_val - min_val) * 200
        pygame.draw.circle(screen, (100, 100, 200), (int(handle_pos), pos[1] + 5), 8)
        
        # Draw label
        text = self.font.render(f"{label}: {value:.2f}", True, self.text_color)
        screen.blit(text, (pos[0], pos[1] - 20))
        
        return slider_rect  # Return for click detection

    def draw_graph(self, screen, data, rect, max_value, label):
        pygame.draw.rect(screen, (255, 255, 255), rect)
        pygame.draw.rect(screen, (200, 200, 200), rect, 1)
        
        if not data:
            return
            
        # Draw graph line
        points = []
        for i, value in enumerate(data):
            x = rect.left + (i / len(data)) * rect.width
            y = rect.bottom - (value / max_value) * rect.height
            points.append((x, y))
            
        if len(points) > 1:
            pygame.draw.lines(screen, self.graph_color, False, points, 2)
            
        # Draw label
        text = self.font.render(label, True, self.text_color)
        screen.blit(text, (rect.left, rect.top - 20))

class TrafficSimulation:
    def __init__(self):
        pygame.init()
        self.width = 1200
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        
        self.roads = []
        self.vehicles = []
        self.hubs = []
        self.drawing = False
        self.current_road_points = []
        self.stats = Stats()
        self.ui = ModernUI(self.width, self.height)
        
        # Time tracking for hub spawning
        self.last_spawn_times = {}  # hub_index -> last_spawn_time
        
        # Simulation speed control (1.0 = normal speed)
        self.time_scale = 0.01
        
    def generate_random_color(self):
        # Generate pastel colors that are visible
        h = np.random.random()
        s = 0.5 + np.random.random() * 0.2  # 0.5-0.7
        v = 0.9 + np.random.random() * 0.1  # 0.9-1.0
        rgb = colorsys.hsv_to_rgb(h, s, v)
        return tuple(int(x * 255) for x in rgb)

    def snap_to_angle(self, start_point, end_point):
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        angle = np.arctan2(dy, dx)
        
        # snap to nearest 30° increment
        snap_angle = np.round(angle / (np.pi/6)) * (np.pi/6)
        
        # calculate new end point using snapped angle
        distance = np.hypot(dx, dy)
        new_x = start_point[0] + distance * np.cos(snap_angle)
        new_y = start_point[1] + distance * np.sin(snap_angle)
        
        return (int(new_x), int(new_y))

    def handle_drawing(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                mouse_pos = event.pos
                if mouse_pos[0] < self.width - 300:
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_h]:  # 'h' key is pressed
                        print("Creating hub at", mouse_pos)
                        self.hubs.append(Hub(position=mouse_pos))
                    else:
                        print("Starting road at", mouse_pos)
                        self.drawing = True
                        self.current_road_points = [mouse_pos]
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1 and self.drawing:
                self.drawing = False
                if len(self.current_road_points) > 1:
                    new_road_index = len(self.roads)
                    # clean up the road points before adding
                    cleaned_points = [self.current_road_points[0]]
                    for i in range(1, len(self.current_road_points)):
                        snapped_point = self.snap_to_angle(cleaned_points[-1], self.current_road_points[i])
                        if np.hypot(snapped_point[0] - cleaned_points[-1][0],
                                  snapped_point[1] - cleaned_points[-1][1]) > 20:  # minimum segment length
                            cleaned_points.append(snapped_point)
                    
                    if len(cleaned_points) > 1:
                        self.roads.append(Road(cleaned_points))
                        
                        # Connect to nearby hubs
                        start_point = cleaned_points[0]
                        end_point = cleaned_points[-1]
                        for hub in self.hubs:
                            if np.hypot(hub.position[0] - start_point[0],
                                      hub.position[1] - start_point[1]) < hub.size:
                                hub.connected_roads.append(new_road_index)
                            if np.hypot(hub.position[0] - end_point[0],
                                      hub.position[1] - end_point[1]) < hub.size:
                                hub.connected_roads.append(new_road_index)
                
                self.current_road_points = []
        elif event.type == pygame.MOUSEMOTION and self.drawing:
            if len(self.current_road_points) > 0:
                # get snapped position for current mouse position
                snapped_pos = self.snap_to_angle(self.current_road_points[0], event.pos)
                # update current points with only start and snapped end
                self.current_road_points = [self.current_road_points[0], snapped_pos]
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1 and self.drawing:
                self.drawing = False
                if len(self.current_road_points) > 1:
                    new_road_index = len(self.roads)
                    self.roads.append(Road(self.current_road_points.copy()))
                    
                    # Connect road to nearby hubs
                    start_point = self.current_road_points[0]
                    end_point = self.current_road_points[-1]
                    
                    for hub in self.hubs:
                        # Connect to start of road
                        if np.hypot(hub.position[0] - start_point[0],
                                  hub.position[1] - start_point[1]) < hub.size:
                            hub.connected_roads.append(new_road_index)
                        
                        # Connect to end of road
                        if np.hypot(hub.position[0] - end_point[0],
                                  hub.position[1] - end_point[1]) < hub.size:
                            hub.connected_roads.append(new_road_index)
                
                self.current_road_points = []
        elif event.type == pygame.MOUSEMOTION and self.drawing:
            if np.hypot(event.pos[0] - self.current_road_points[-1][0],
                       event.pos[1] - self.current_road_points[-1][1]) > 10:
                self.current_road_points.append(event.pos)

    def get_road_angle(self, road, position):
        idx = int(position * (len(road.points) - 1))
        next_idx = min(idx + 1, len(road.points) - 1)
        dx = road.points[next_idx][0] - road.points[idx][0]
        dy = road.points[next_idx][1] - road.points[idx][1]
        return np.arctan2(dy, dx)

    def draw_vehicle(self, vehicle, position):
        road = self.roads[vehicle.road_index]
        angle = self.get_road_angle(road, vehicle.position)
        
        # Create rectangle points
        points = [
            (-vehicle.length/2, -vehicle.width/2),
            (vehicle.length/2, -vehicle.width/2),
            (vehicle.length/2, vehicle.width/2),
            (-vehicle.length/2, vehicle.width/2)
        ]
        
        # Rotate points
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        rotated_points = [
            (x * cos_a - y * sin_a + position[0],
             x * sin_a + y * cos_a + position[1])
            for x, y in points
        ]
        
        # Draw filled polygon
        pygame.gfxdraw.filled_polygon(self.screen, 
                                    [(int(x), int(y)) for x, y in rotated_points],
                                    vehicle.color)
        
        # Draw outline
        pygame.gfxdraw.aapolygon(self.screen,
                                [(int(x), int(y)) for x, y in rotated_points],
                                (50, 50, 50))

    def draw(self):
        # Main simulation area
        self.screen.fill((255, 255, 255))
        
        # Draw existing roads
        for road in self.roads:
            if len(road.points) > 1:
                pygame.draw.lines(self.screen, (100, 100, 100), False, road.points, road.width)
        
        # Draw current road being drawn
        if len(self.current_road_points) > 1:
            pygame.draw.lines(self.screen, (150, 150, 150), False, self.current_road_points, 20)
        
        # Draw hubs
        for i, hub in enumerate(self.hubs):
            pygame.draw.rect(self.screen, hub.color,
                           (hub.position[0] - hub.size//2,
                            hub.position[1] - hub.size//2,
                            hub.size, hub.size))
        
        # Draw vehicles
        for vehicle in self.vehicles:
            road = self.roads[vehicle.road_index]
            if len(road.points) > 1:
                idx = int(vehicle.position * (len(road.points) - 1))
                next_idx = min(idx + 1, len(road.points) - 1)
                t = vehicle.position * (len(road.points) - 1) - idx
                
                pos = (
                    road.points[idx][0] * (1 - t) + road.points[next_idx][0] * t,
                    road.points[idx][1] * (1 - t) + road.points[next_idx][1] * t
                )
                self.draw_vehicle(vehicle, pos)
        
        # Draw UI panel
        pygame.draw.rect(self.screen, self.ui.background_color, self.ui.panel_rect)
        
        # Draw statistics
        stats_text = [
            f"Cars: {len(self.vehicles)}",
            f"Avg Speed: {np.mean([v.speed for v in self.vehicles]) if self.vehicles else 0:.1f}",
            f"Simulation Speed: {self.time_scale:.1f}x"
        ]
        
        for i, text in enumerate(stats_text):
            surface = self.ui.font.render(text, True, self.ui.text_color)
            self.screen.blit(surface, (self.width - 280, 20 + i * 30))
        
        # Draw graphs
        speed_graph_rect = pygame.Rect(self.width - 280, 120, 260, 100)
        self.ui.draw_graph(self.screen, self.stats.speed_history, speed_graph_rect, 5, "Average Speed")
        
        count_graph_rect = pygame.Rect(self.width - 280, 270, 260, 100)
        self.ui.draw_graph(self.screen, self.stats.car_count_history, count_graph_rect, 
                          max(max(self.stats.car_count_history, default=1), 1), "Car Count")
        
        # Draw simulation speed slider
        speed_slider_pos = (self.width - 280, 400)
        self.ui.draw_slider(self.screen, speed_slider_pos, self.time_scale, 0.1, 3.0, "Sim Speed")
        
        # Draw hub controls
        for i, hub in enumerate(self.hubs):
            slider_pos = (self.width - 280, 460 + i * 60)
            self.ui.draw_slider(self.screen, slider_pos, hub.spawn_rate, 0, 2, f"Hub {i+1} Rate")
        
        pygame.display.flip()
        
        pygame.display.flip()

    def spawn_vehicle(self, road_index):
        # Check if there's enough space at the start of the road
        can_spawn = True
        for vehicle in self.vehicles:
            if vehicle.road_index == road_index and vehicle.position < 0.2:
                can_spawn = False
                break
        
        if can_spawn:
            print(f"Spawning vehicle on road {road_index}")  # Debug print
            self.vehicles.append(Vehicle(
                position=0,
                speed=1,
                road_index=road_index,
                color=self.generate_random_color()
            ))

    def spawn_vehicle_from_hub(self, hub_index):
        hub = self.hubs[hub_index]
        if not hub.connected_roads:
            return
            
        road_index = np.random.choice(hub.connected_roads)
        road = self.roads[road_index]
        
        # Check if road start is near hub
        road_start = road.points[0]
        road_end = road.points[-1]
        
        position = 0
        if np.hypot(road_end[0] - hub.position[0],
                   road_end[1] - hub.position[1]) < hub.size:
            position = 1
        
        # Only spawn if there's enough space
        can_spawn = True
        for vehicle in self.vehicles:
            if vehicle.road_index == road_index:
                if position == 0 and vehicle.position < 0.2:
                    can_spawn = False
                    break
                elif position == 1 and vehicle.position > 0.8:
                    can_spawn = False
                    break
        
        if can_spawn:
            self.vehicles.append(Vehicle(
                position=position,
                speed=1,
                road_index=road_index,
                color=self.generate_random_color()
            ))

    def get_distance_to_next_vehicle(self, vehicle):
        road = self.roads[vehicle.road_index]
        min_distance = float('inf')
        
        # Check distance to vehicles on same road
        for other in self.vehicles:
            if other != vehicle and other.road_index == vehicle.road_index:
                dist = other.position - vehicle.position
                if dist > 0:  # only check vehicles ahead
                    min_distance = min(min_distance, dist * len(road.points))
        
        # Check vehicles on connected roads if near end
        if vehicle.position > 0.8:  # check next road when near end
            end_point = road.points[-1]
            connected_roads = [
                (i, r) for i, r in enumerate(self.roads)
                if np.hypot(r.points[0][0] - end_point[0],
                          r.points[0][1] - end_point[1]) < road.width
            ]
            
            for next_road_idx, next_road in connected_roads:
                for other in self.vehicles:
                    if other.road_index == next_road_idx and other.position < 0.2:
                        remaining_dist = (1 - vehicle.position) * len(road.points)
                        next_road_dist = other.position * len(next_road.points)
                        min_distance = min(min_distance, remaining_dist + next_road_dist)
        
        return min_distance

    def update_vehicles(self):
        # Update each vehicle according to N-S rules
        for vehicle in list(self.vehicles):  # Create copy of list for safe removal
            road = self.roads[vehicle.road_index]
            
            # 1. Acceleration
            if vehicle.speed < road.max_speed:
                vehicle.speed += 1
            
            # 2. Slowing down (due to other cars)
            distance_ahead = self.get_distance_to_next_vehicle(vehicle)
            if distance_ahead != float('inf'):
                vehicle.speed = min(vehicle.speed, int(distance_ahead - 1))  # keep safe distance
            
            # 3. Randomization (dawdling)
            if vehicle.speed > 0 and np.random.random() < 0.3:  # 30% chance to slow down
                vehicle.speed = max(0, vehicle.speed - 1)
            
            # 4. Car movement (slower progression for better visualization)
            vehicle.position += (vehicle.speed / len(road.points)) * 0.2 * self.time_scale  # scaled movement with time scale
            
            # Handle road transitions or vehicle removal
            if vehicle.position >= 1:
                # Find connected roads
                end_point = road.points[-1]
                connected_roads = [
                    (i, r) for i, r in enumerate(self.roads)
                    if np.hypot(r.points[0][0] - end_point[0],
                              r.points[0][1] - end_point[1]) < road.width
                ]
                
                if connected_roads:
                    # Choose random connected road
                    new_road_index, _ = connected_roads[np.random.randint(len(connected_roads))]
                    vehicle.road_index = new_road_index
                    vehicle.position = 0
                else:
                    # Remove vehicle if no connected road
                    if vehicle in self.vehicles:  # Check if vehicle still exists
                        self.vehicles.remove(vehicle)
            elif vehicle.position < 0:  # Handle reverse movement if we add it later
                if vehicle in self.vehicles:
                    self.vehicles.remove(vehicle)

    def update_hubs(self):
        current_time = pygame.time.get_ticks() / 1000  # Convert to seconds
        
        for i, hub in enumerate(self.hubs):
            last_spawn = self.last_spawn_times.get(i, 0)
            if current_time - last_spawn > 1 / hub.spawn_rate:
                self.spawn_vehicle_from_hub(i)
                self.last_spawn_times[i] = current_time

    def get_safe_mouse_position(self):
        # Get mouse position but ensure it's in the main area (not in UI panel)
        mouse_pos = pygame.mouse.get_pos()
        if mouse_pos[0] >= self.width - 300:  # If in UI panel, return None
            return None
        return mouse_pos

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    print(f"Key pressed: {event.key}")  # Debug print
                    if event.key == pygame.K_c:  # 'c' to clear everything
                        self.roads = []
                        self.vehicles = []
                        self.hubs = []
                    elif event.key == pygame.K_SPACE:  # space to manually spawn a vehicle
                        print("Manual vehicle spawn attempted")  # Debug print
                        if self.roads:
                            if self.hubs:  # If there are hubs, spawn from a random hub
                                hub_index = np.random.randint(len(self.hubs))
                                self.spawn_vehicle_from_hub(hub_index)
                            else:  # Otherwise spawn at start of a random road
                                self.spawn_vehicle(np.random.randint(len(self.roads)))
                    elif event.key == pygame.K_h:  # 'h' to create hub at mouse position
                        mouse_pos = self.get_safe_mouse_position()
                        if mouse_pos:
                            print(f"Creating hub at {mouse_pos}")  # Debug print
                            self.hubs.append(Hub(position=mouse_pos))
                
                self.handle_drawing(event)
                
                # Handle slider interactions
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = pygame.mouse.get_pos()
                    if mouse_pos[0] > self.width - 300:
                        # Check simulation speed slider
                        speed_slider_rect = pygame.Rect(self.width - 280, 400, 200, 10)
                        if speed_slider_rect.collidepoint(mouse_pos):
                            value = (mouse_pos[0] - (self.width - 280)) / 200 * 2.9 + 0.1  # Range 0.1-3.0
                            self.time_scale = max(0.1, min(3.0, value))
                            print(f"Simulation speed set to {self.time_scale}x")  # Debug print
                        
                        # Check hub sliders
                        for i, hub in enumerate(self.hubs):
                            slider_rect = pygame.Rect(self.width - 280, 460 + i * 60, 200, 10)
                            if slider_rect.collidepoint(mouse_pos):
                                value = (mouse_pos[0] - (self.width - 280)) / 200 * 2
                                hub.spawn_rate = max(0, min(2, value))
                                print(f"Hub {i} spawn rate set to {hub.spawn_rate}")  # Debug print
            
            # Update simulation
            self.update_vehicles()
            self.update_hubs()
            self.stats.update(self.vehicles)
            self.draw()
            self.clock.tick(60)
            
            self.update_vehicles()
            self.update_hubs()
            self.stats.update(self.vehicles)
            self.draw()
            self.clock.tick(60)
        
        pygame.quit()

if __name__ == "__main__":
    sim = TrafficSimulation()
    sim.run()
