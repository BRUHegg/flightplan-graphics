import pygame as pg

ASPECT_RATIO = 488/751
WND_HEIGHT = 900
WND_WIDTH = ASPECT_RATIO * WND_HEIGHT
BACKGND_CLR_LIGHT = (194, 156, 117)
BACKGND_CLR = (177, 144, 109)
LSK_CLR = (56, 58, 57)
LSK_CLR_LT = (76, 78, 77)
WHITE = (255, 255, 255)
pg.init()
screen = pg.display.set_mode((WND_WIDTH, WND_HEIGHT))

LSK_OFFS = 0.1
SCR_OFFS_OUTER = 0.04
SCR_RECT_OUTER_W = WND_WIDTH * (1 - 2 * LSK_OFFS)
SCR_RECT_OUTER_H = WND_HEIGHT * 0.43
LSK_KEY_OFFS_LAT = 0.01 * WND_WIDTH
LSK_KEY_OFFS_VERT = 8 * LSK_KEY_OFFS_LAT
LSK_WIDTH = 0.08 * WND_WIDTH
LSK_HEIGHT = 0.7 * LSK_WIDTH
LSK_L_WIDTH = LSK_HEIGHT * 0.2
LSK_OFFS_START = WND_HEIGHT * 0.12
LSK_LINE_OFFS = 0.02 * WND_WIDTH
LSK_INN_OFFS = WND_WIDTH * 0.01
KEY_MAIN_START_X = LSK_OFFS * WND_WIDTH
KEY_MAIN_START_Y = 0.51 * WND_HEIGHT
KEY_MAIN_WIDTH = LSK_WIDTH * 1.5
KEY_MAIN_HEIGHT = LSK_HEIGHT * 1.47
KEY_MAIN_LAT_OFFS = KEY_MAIN_WIDTH * 1.06
KEY_MAIN_VERT_OFFS = KEY_MAIN_HEIGHT * 1.08

LETTER_KEY_WIDTH = 0.075 * WND_WIDTH
LETTER_KEY_HEIGHT = LETTER_KEY_WIDTH
LETTER_KEY_START_X = 0.44 * WND_WIDTH
LETTER_KEY_START_Y = 0.63 * WND_HEIGHT
LETTER_KEY_LAT_OFFS = LETTER_KEY_WIDTH * 1.24
LETTER_KEY_VERT_OFFS = LETTER_KEY_WIDTH * 1.24


save = False

def draw_btn(x, y, w, h, text=""):
    curr_rect = pg.Rect(x, y, w, h)
    curr_rect_inn = pg.Rect(x+LSK_INN_OFFS, y+LSK_INN_OFFS, 
                            w-LSK_INN_OFFS*2, h-LSK_INN_OFFS*2)
    pg.draw.rect(screen, LSK_CLR, curr_rect, 0, 5)
    pg.draw.rect(screen, LSK_CLR_LT, curr_rect_inn, 0, 5)

    if text == "":
        l_y = curr_rect[1] + curr_rect[3] * 0.5
        pg.draw.line(screen, WHITE, (curr_rect[0]+LSK_LINE_OFFS, l_y),
                    (curr_rect[0]+curr_rect[2]-LSK_LINE_OFFS, l_y),int(LSK_L_WIDTH))

def draw_lsk(x, y):
    draw_btn(x, y, LSK_WIDTH, LSK_HEIGHT)

while 1:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            exit()
    
    screen.fill(BACKGND_CLR_LIGHT)

    pg.draw.rect(screen, BACKGND_CLR, 
                (WND_WIDTH * LSK_OFFS, WND_HEIGHT * SCR_OFFS_OUTER, 
                 SCR_RECT_OUTER_W, SCR_RECT_OUTER_H), 25, 15)
    
    curr_rect_l = pg.Rect(LSK_KEY_OFFS_LAT, LSK_OFFS_START, LSK_WIDTH, LSK_HEIGHT)
    curr_rect_r = pg.Rect(WND_WIDTH-LSK_KEY_OFFS_LAT-LSK_WIDTH, LSK_OFFS_START, LSK_WIDTH, LSK_HEIGHT)
    for i in range(6):
        draw_lsk(curr_rect_l[0], curr_rect_l[1])
        draw_lsk(curr_rect_r[0], curr_rect_r[1])
        
        curr_rect_l[1] += LSK_KEY_OFFS_VERT
        curr_rect_r[1] += LSK_KEY_OFFS_VERT

    curr_y = KEY_MAIN_START_Y
    for i in range(2):
        curr_x = KEY_MAIN_START_X
        for j in range(5):
            draw_btn(curr_x, curr_y, KEY_MAIN_WIDTH, KEY_MAIN_HEIGHT, "A")
            curr_x += KEY_MAIN_LAT_OFFS
        curr_y += KEY_MAIN_VERT_OFFS

    for i in range(2):
        curr_x = KEY_MAIN_START_X
        for j in range(2):
            draw_btn(curr_x, curr_y, KEY_MAIN_WIDTH, KEY_MAIN_HEIGHT, "A")
            curr_x += KEY_MAIN_LAT_OFFS
        curr_y += KEY_MAIN_VERT_OFFS

    curr_y = LETTER_KEY_START_Y
    for i in range(6):
        curr_x = LETTER_KEY_START_X
        for j in range(5):
            draw_btn(curr_x, curr_y, LETTER_KEY_WIDTH, LETTER_KEY_HEIGHT, "A")
            curr_x += LETTER_KEY_LAT_OFFS
        curr_y += LETTER_KEY_VERT_OFFS

    if save:
        pg.image.save(screen, "cdu.png")
        break
    pg.display.update()