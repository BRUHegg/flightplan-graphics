import pygame as pg

ASPECT_RATIO = 488/751
WND_HEIGHT = 900
WND_WIDTH = ASPECT_RATIO * WND_HEIGHT
BACKGND_CLR_LIGHT = (194, 156, 117)
BACKGND_CLR = (177, 144, 109)
MAIN_KEY_LIGHT = (207, 181, 148)
MAIN_KEY_MED_LIGHT = (197, 171, 138)
LSK_CLR = (56, 58, 57)
LSK_CLR_LT = (76, 78, 77)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
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

MAIN_KEY_FNT = 16
KEY_MAIN_START_X = LSK_OFFS * WND_WIDTH
KEY_MAIN_START_Y = 0.51 * WND_HEIGHT
KEY_MAIN_WIDTH = LSK_WIDTH * 1.5
KEY_MAIN_HEIGHT = LSK_HEIGHT * 1.47
KEY_MAIN_LAT_OFFS = KEY_MAIN_WIDTH * 1.06
KEY_MAIN_VERT_OFFS = KEY_MAIN_HEIGHT * 1.08

LETTER_KEY_FNT = 32
LETTER_KEY_WIDTH = 0.075 * WND_WIDTH
LETTER_KEY_HEIGHT = LETTER_KEY_WIDTH
LETTER_KEY_START_X = 0.44 * WND_WIDTH
LETTER_KEY_START_Y = 0.63 * WND_HEIGHT
LETTER_KEY_LAT_OFFS = LETTER_KEY_WIDTH * 1.24
LETTER_KEY_VERT_OFFS = LETTER_KEY_WIDTH * 1.24

NUM_KEY_START_X = KEY_MAIN_START_X
NUM_KEY_START_Y = WND_HEIGHT * 0.75
NUM_KEY_LAT_OFFS = LETTER_KEY_LAT_OFFS * 1.15

EXEC_BTN_X = WND_WIDTH * 0.78
EXEC_BTN_Y = WND_HEIGHT * 0.582
EXEC_BTN_W = LSK_WIDTH * 1.2
EXEC_BTN_H = LSK_HEIGHT * 1.08
EXEC_BTN_FNT = 14

EXEC_LT_X = EXEC_BTN_X
EXEC_LT_Y = EXEC_BTN_Y * 0.96
EXEC_LT_W = EXEC_BTN_W
EXEC_LT_H = EXEC_BTN_H * 0.37

MAIN_KEY_RECT_X = WND_WIDTH * 0.09
MAIN_KEY_RECT_Y = WND_HEIGHT * 0.505
MAIN_KEY_RECT_W1 = WND_WIDTH * 0.84
MAIN_KEY_RECT_H1 = WND_HEIGHT * 0.123
MAIN_KEY_RECT_W2 = MAIN_KEY_RECT_H1 * 1.42
MAIN_KEY_RECT_H2 = WND_HEIGHT * 0.236


TEXT_OFFS = WND_HEIGHT * 0.001


save_cdu = True
save_keys = False

def draw_ctrd_text(x, y, v_offs, s, fnt_sz):
    if(s != ""):
        s_lines = list(s.split("\n"))
        
        font = pg.font.Font("BoeingFont.ttf", fnt_sz)
        text = font.render(s_lines[0], True, WHITE)
        text_rect = text.get_rect()
        t_h = text_rect.height
        y_start = y - (t_h * (len(s_lines)//2) + v_offs * max(0, (len(s_lines)//2-1)))
        if(len(s_lines) > 1):
            if(len(s_lines) % 2 == 0):
                y_start -= v_offs / 2
            else:
                y_start -= t_h / 2
        else:
            y_start = y - t_h/2
        for i in s_lines:
            text = font.render(i, True, WHITE)
            text_rect = text.get_rect(center=(x, y_start+t_h/2))
            screen.blit(text, text_rect)
            y_start += t_h+v_offs

def draw_depth_rect(x, y, w, h, inn_offs, c_out, c_inn):
    curr_rect = pg.Rect(x, y, w, h)
    curr_rect_inn = pg.Rect(x+inn_offs, y+inn_offs, 
                            w-inn_offs*2, h-inn_offs*2)
    pg.draw.rect(screen, c_out, curr_rect, 0, 5)
    pg.draw.rect(screen, c_inn, curr_rect_inn, 0, 5)

def draw_btn(x, y, w, h, s="", fnt_sz=25):
    curr_rect = pg.Rect(x, y, w, h)
    curr_rect_inn = pg.Rect(x+LSK_INN_OFFS, y+LSK_INN_OFFS, 
                            w-LSK_INN_OFFS*2, h-LSK_INN_OFFS*2)
    pg.draw.rect(screen, LSK_CLR, curr_rect, 0, 5)
    pg.draw.rect(screen, LSK_CLR_LT, curr_rect_inn, 0, 5)

    if s == "":
        l_y = curr_rect[1] + curr_rect[3] * 0.5
        pg.draw.line(screen, WHITE, (curr_rect[0]+LSK_LINE_OFFS, l_y),
                    (curr_rect[0]+curr_rect[2]-LSK_LINE_OFFS, l_y),int(LSK_L_WIDTH))
    else:
        btn_ctr = (x + w / 2, y + h / 2)
        draw_ctrd_text(btn_ctr[0], btn_ctr[1], TEXT_OFFS, s, fnt_sz)

def draw_round_btn(x, y, r, s="", fnt_sz=25):
    curr_ctr = (x + r, y + r)
    r1 = r
    r2 = r1 - LSK_INN_OFFS

    pg.draw.circle(screen, LSK_CLR, curr_ctr, r1)
    pg.draw.circle(screen, LSK_CLR_LT, curr_ctr, r2)

    draw_ctrd_text(curr_ctr[0], curr_ctr[1], 0, s, fnt_sz)

def draw_lsk(x, y):
    draw_btn(x, y, LSK_WIDTH, LSK_HEIGHT)

main_btn_labels = ["INIT\nREF", "RTE", "DEP\nARR", "ALTN", "VNAV", "FIX", "LEGS", "HOLD", "FMC\nCOMM",
                   "PROG", "MENU", "NAV\nRAD", "PREV\nPAGE", "NEXT\nPAGE"]

btn_labels = []

for i in main_btn_labels:
    btn_labels.append((i, MAIN_KEY_FNT))
for i in range(26):
    btn_labels.append((chr(ord("A")+i), LETTER_KEY_FNT))
btn_labels.append(("SP", MAIN_KEY_FNT))
btn_labels.append(("DEL", MAIN_KEY_FNT))
btn_labels.append(("/", LETTER_KEY_FNT))
btn_labels.append(("CLR", MAIN_KEY_FNT))
for i in range(1, 10):
    btn_labels.append((str(i), LETTER_KEY_FNT))
btn_labels.append((".", LETTER_KEY_FNT))
btn_labels.append(("0", LETTER_KEY_FNT))
btn_labels.append(("+/-", MAIN_KEY_FNT))
btn_labels.append(("EXEC", EXEC_BTN_FNT))

while 1:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            exit()
    
    if not save_keys:
        screen.fill(BACKGND_CLR_LIGHT)
        pg.draw.rect(screen, BACKGND_CLR, 
                (WND_WIDTH * LSK_OFFS, WND_HEIGHT * SCR_OFFS_OUTER, 
                 SCR_RECT_OUTER_W, SCR_RECT_OUTER_H), 25, 15)
    
        draw_depth_rect(MAIN_KEY_RECT_X, MAIN_KEY_RECT_Y, 
                        MAIN_KEY_RECT_W2, MAIN_KEY_RECT_H2, WND_WIDTH * 0.005, MAIN_KEY_MED_LIGHT, 
                        MAIN_KEY_LIGHT)
        draw_depth_rect(MAIN_KEY_RECT_X, MAIN_KEY_RECT_Y, 
                        MAIN_KEY_RECT_W1, MAIN_KEY_RECT_H1, WND_WIDTH * 0.005, MAIN_KEY_MED_LIGHT, 
                        MAIN_KEY_LIGHT)
        draw_depth_rect(EXEC_LT_X, EXEC_LT_Y, EXEC_LT_W, EXEC_LT_H, WND_WIDTH * 0.004, 
                    MAIN_KEY_MED_LIGHT, BLACK)
    else:
        screen = pg.Surface((WND_WIDTH, WND_HEIGHT), pg.SRCALPHA)
        screen.fill((255,0,0, 0))
    
    if save_cdu:
        pg.image.save(screen, "cdu_back.png")
        break
    
    curr_rect_l = pg.Rect(LSK_KEY_OFFS_LAT, LSK_OFFS_START, LSK_WIDTH, LSK_HEIGHT)
    curr_rect_r = pg.Rect(WND_WIDTH-LSK_KEY_OFFS_LAT-LSK_WIDTH, LSK_OFFS_START, LSK_WIDTH, LSK_HEIGHT)
    for i in range(6):
        draw_lsk(curr_rect_l[0], curr_rect_l[1])
        draw_lsk(curr_rect_r[0], curr_rect_r[1])
        
        curr_rect_l[1] += LSK_KEY_OFFS_VERT
        curr_rect_r[1] += LSK_KEY_OFFS_VERT

    k = 0
    curr_y = KEY_MAIN_START_Y
    for i in range(2):
        curr_x = KEY_MAIN_START_X
        for j in range(5):
            draw_btn(curr_x, curr_y, KEY_MAIN_WIDTH, KEY_MAIN_HEIGHT, btn_labels[k][0],
                        btn_labels[k][1])
            curr_x += KEY_MAIN_LAT_OFFS
            k+=1
        curr_y += KEY_MAIN_VERT_OFFS

    for i in range(2):
        curr_x = KEY_MAIN_START_X
        for j in range(2):
            draw_btn(curr_x, curr_y, KEY_MAIN_WIDTH, KEY_MAIN_HEIGHT, btn_labels[k][0],
                        btn_labels[k][1])
            curr_x += KEY_MAIN_LAT_OFFS
            k+=1
        curr_y += KEY_MAIN_VERT_OFFS

    curr_y = LETTER_KEY_START_Y
    for i in range(6):
        curr_x = LETTER_KEY_START_X
        for j in range(5):
            draw_btn(curr_x, curr_y, LETTER_KEY_WIDTH, LETTER_KEY_HEIGHT, btn_labels[k][0],
                        btn_labels[k][1])
            curr_x += LETTER_KEY_LAT_OFFS
            k += 1
        curr_y += LETTER_KEY_VERT_OFFS

    curr_y = NUM_KEY_START_Y
    for i in range(4):
        curr_x = NUM_KEY_START_X
        for j in range(3):
            draw_round_btn(curr_x, curr_y, LETTER_KEY_WIDTH/2, btn_labels[k][0],
                        btn_labels[k][1])
            k += 1
            curr_x += NUM_KEY_LAT_OFFS
        curr_y += LETTER_KEY_VERT_OFFS

    draw_btn(EXEC_BTN_X, EXEC_BTN_Y, EXEC_BTN_W, EXEC_BTN_H, btn_labels[k][0],
                        btn_labels[k][1])

    if save_keys:
        pg.image.save(screen, "cdu_keys.png")
        break
    pg.display.update()