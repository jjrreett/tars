import pygame
import math
from collections import deque

def lerp(x1: float, x2: float, y1: float, y2: float, x: float):
    """Perform linear interpolation for x between (x1,y1) and (x2,y2) """

    return ((y2 - y1) * x + x2 * y1 - x1 * y2) / (x2 - x1)



class LivePlottingApp:
    def __init__(self, width=800, height=600, mod=1):
        pygame.init()
        self.width = width
        self.height = height
        self.window = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Live Plotting App")
        self.clock = pygame.time.Clock()
        self.data_queues = []
        self.names = []
        self.colors = []
        self.max_length = width//mod
        self.mod=mod

    def set_names(self, names):
        self.names = names
        self.data_queues = [deque(maxlen=self.max_length) for _ in names]

    def set_colors(self, colors):
        self.colors = colors

    def feed(self, data_list):
        for data_queue, data_point in zip(self.data_queues, data_list):
            data_queue.append(data_point)
        self.draw_plot()

    def draw_plot(self):
        self.window.fill((255, 255, 255))  # Clear the screen with white color
        for data_queue, color in zip(self.data_queues, self.colors):
            if len(data_queue) > 1:
                max_d = max(data_queue)
                min_d = min(data_queue)

                abs_max = max(abs(max_d), abs(min_d))

                if abs_max == 0:
                    abs_max = 1

                pygame.draw.aalines(
                    self.window,
                    color,
                    False,
                    [(i*self.mod, lerp(-abs_max, abs_max, self.height, 0, d)) for i, d in enumerate(data_queue)],
                )
        pygame.display.flip()

    def is_quit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        return False

    def close(self):
        pygame.display.quit()
        pygame.quit()


if __name__ == "__main__":
    app = LivePlottingApp(width=1200, mod=2)
    app.set_names(["Sine Wave 1", "Sine Wave 2"])
    app.set_colors([(0, 0, 255), (255, 0, 0)])

    t = 0
    while not (app.is_quit()):
        data_points = [
            math.sin(t),  3 * math.cos(t)**2
        ]
        app.feed(data_points)
        app.clock.tick(60)
        t += 0.1
    app.close()
