# run.py
import pygame
import sys
from agent import Agent
from Environment import Environment

# Constants
WINDOW_WIDTH, WINDOW_HEIGHT = 800, 600
GRID_SIZE = 40
STATUS_WIDTH = 300
BACKGROUND_COLOR = (255, 255, 255)
BARRIER_COLOR = (0, 0, 0)
TASK_COLOR = (255, 0, 0)
TEXT_COLOR = (0, 0, 0)
BUTTON_COLOR = (0, 200, 0)
BUTTON_HOVER_COLOR = (0, 255, 0)
BUTTON_TEXT_COLOR = (255, 255, 255)
MOVEMENT_DELAY = 200  # Milliseconds

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH + STATUS_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Pygame AI Grid Simulation - IDA* Pathfinding")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    environment = Environment(WINDOW_WIDTH, WINDOW_HEIGHT, GRID_SIZE, num_tasks=5, num_barriers=15)
    agent = Agent(environment, GRID_SIZE)
    all_sprites = pygame.sprite.Group()
    all_sprites.add(agent)

    button_width, button_height = 100, 50
    button_x = WINDOW_WIDTH + (STATUS_WIDTH - button_width) // 2
    button_y = WINDOW_HEIGHT // 2 - button_height // 2
    button_rect = pygame.Rect(button_x, button_y, button_width, button_height)
    simulation_started = False
    last_move_time = pygame.time.get_ticks()

    running = True
    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if not simulation_started and event.type == pygame.MOUSEBUTTONDOWN:
                if button_rect.collidepoint(event.pos):
                    simulation_started = True
                    if environment.task_locations:
                        agent.find_nearest_task()

        screen.fill(BACKGROUND_COLOR)
        for x in range(environment.columns):
            for y in range(environment.rows):
                rect = pygame.Rect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE)
                pygame.draw.rect(screen, (200, 200, 200), rect, 1)
        
        for (bx, by) in environment.barrier_locations:
            pygame.draw.rect(screen, BARRIER_COLOR, pygame.Rect(bx * GRID_SIZE, by * GRID_SIZE, GRID_SIZE, GRID_SIZE))

        for (tx, ty), task_number in environment.task_locations.items():
            task_rect = pygame.Rect(tx * GRID_SIZE, ty * GRID_SIZE, GRID_SIZE, GRID_SIZE)
            pygame.draw.rect(screen, TASK_COLOR, task_rect)
            task_surface = font.render(str(task_number), True, (255, 255, 255))
            task_text_rect = task_surface.get_rect(center=task_rect.center)
            screen.blit(task_surface, task_text_rect)

        all_sprites.draw(screen)

        
        algorithm_text = "Algorithm: IDA*"
        status_x = WINDOW_WIDTH + 10
        task_status_text = f"Tasks Completed: {agent.task_completed}"
        position_text = f"Position: {agent.position}"
        completed_tasks_text = f"Completed Tasks: {agent.completed_tasks}"
        path_cost_text = f"Path Cost: {len(agent.explored_path)}"
        
        status_surface = font.render(task_status_text, True, TEXT_COLOR)
        position_surface = font.render(position_text, True, TEXT_COLOR)
        completed_tasks_surface = font.render(completed_tasks_text, True, TEXT_COLOR)
        path_cost_surface = font.render(path_cost_text, True, TEXT_COLOR)
        algorithm_surface = font.render(algorithm_text, True, TEXT_COLOR)

        screen.blit(algorithm_surface, (status_x, 20))
        screen.blit(status_surface, (status_x, 50))
        screen.blit(position_surface, (status_x, 80))
        screen.blit(completed_tasks_surface, (status_x, 110))
        screen.blit(path_cost_surface, (status_x, 140))

        if not simulation_started:
            pygame.draw.rect(screen, BUTTON_COLOR, button_rect)
            button_text = font.render("Start", True, BUTTON_TEXT_COLOR)
            text_rect = button_text.get_rect(center=button_rect.center)
            screen.blit(button_text, text_rect)
        else:
            if pygame.time.get_ticks() - last_move_time > MOVEMENT_DELAY:
                if not agent.moving and environment.task_locations:
                    agent.find_nearest_task()
                elif agent.moving:
                    agent.move()
                last_move_time = pygame.time.get_ticks()
        
        pygame.draw.line(screen, (0, 0, 0), (WINDOW_WIDTH, 0), (WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()