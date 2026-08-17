import pygame as pg

WND_WIDTH = WND_HEIGHT = 200
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
CYAN = (0, 206, 255)

pg.init()
screen = pg.display.set_mode((WND_WIDTH, WND_HEIGHT), pg.SRCALPHA)

screen.fill((0,0,0, 255))

wpt_tri_rel = [(0.8, 0.8), (0.2, 0.8), (0.5, 0.8 - (3**0.5) * 0.3)]
wpt_tri_points = []
for (x, y) in wpt_tri_rel:
    wpt_tri_points.append((x * WND_WIDTH, y * WND_HEIGHT))

def draw_smooth_line(sfc, p_start, p_end, color, thickness):
    pg.draw.line(sfc, color, p_start, p_end, thickness)
    radius = 5
    pg.draw.circle(sfc, color, p_start, radius)
    pg.draw.circle(sfc, color, p_end, radius)

wpt_line_thick = 12

while 1:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            exit()

    for i in range(len(wpt_tri_points)):
       draw_smooth_line(screen, wpt_tri_points[i - 1], wpt_tri_points[i], 
                        CYAN, wpt_line_thick)
    pg.image.save(screen, "waypoint_triang.png")
    break
    pg.display.update()
