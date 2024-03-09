import numpy as np
import pygame
from pygame.locals import *
import sys
import math
import time
SCREEN_SIZE = (1000, 800)
OFFSET = 100
STEER_RATE = (SCREEN_SIZE[0]/2)/(math.pi/4.5)
DRAW_SCALE = 20#計算上の単位(m)から描画単位(px)への変換
wheel_len = 0.38
wheel_wid = 0.2
dt = 0.01

"""
使い方
- center_attachの切り替えにより、視点を道路固定にするか車両固定にするか選べるよ
- draw_road()に色々な経路を合体・自作して代入すると色々な道が楽しめるよ
- draw_road()は経路全体を折線を中心とした四角形の集合として描画するよ
- draw_road()のnumは1か2だよ 2の場合は中央線が書かれるよ
"""

###視点移動の有無はここで切り替え！###
center_attach = True
################################
attach_pos = np.array([0.0,10.0])

def Rot(theta):
  ret = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
  return ret

def vec(theta):
    dir = np.array((-math.sin(theta),math.cos(theta)))
    return dir

def Trans(pos,theta,p):
    #-theta,-pだけposの座標をずらす
    p = np.array(p)
    newpos = np.array(pos)
    newpos = np.dot(Rot(-theta),newpos-p)
    return newpos+attach_pos

def XX(x):
    return x*DRAW_SCALE+SCREEN_SIZE[0]/2

def YY(y):
    return SCREEN_SIZE[1]-OFFSET-y*DRAW_SCALE

def XY(pos):
    return [XX(pos[0]),YY(pos[1])]


class wheel:
    def __init__(self,rel_x,rel_y,carpos,cartheta):
        carx=carpos[0]
        cary=carpos[1]
        self.rel_pos = np.array([rel_x,rel_y])
        self.abs_pos = carpos + self.rel_pos
        self.theta = 0 #ステア角
        self.abstheta = self.theta + cartheta # 絶対座標に対する角度
        

class Car:
    def __init__(self,pos,vel=0.0):
        self.pos = np.array(pos)
        #後輪間中点をposとしよう(内輪差を表現するためには後輪側がv*dt進んだと考えた方が適切っぽい)
        self.theta = 0
        #諸元はプリウスを参考
        self.wid = 1.78
        self.length = 4.6
        self.wh_bs = 2.75
        self.vel = vel
        self.lf = wheel(-self.wid/2,self.wh_bs,self.pos,self.theta)
        self.rf = wheel(self.wid/2,self.wh_bs,self.pos,self.theta)
        self.lb = wheel(-self.wid/2,0,self.pos,self.theta)
        self.rb = wheel(self.wid/2,0,self.pos,self.theta)
    def step_move(self,steer):
        #全体のpos,thetaをsteerとvelに基づいて変換する&タイヤとかにも変更を波及させる
        weigh_rate=0.6
        self.theta += self.vel*dt*(math.sin(steer)*weigh_rate+math.tan(steer)*weigh_rate)/self.wh_bs
        self.pos += self.vel*dt*vec(self.theta)
        
        # sinではなくtanとする文献もある
        # 荷重分布次第で前輪と後輪どちらに準拠するかが変わってくる　前者ならsin、後者ならtan
        # 一般的なステア角度(pi/5)付近までならそんなに変わらない　荷重分布は6:4なので、それに基づいて重み平均を取る

        #タイヤへのapply
        self.lf.theta = steer
        self.lf.abstheta = steer + self.theta
        self.rf.theta = steer
        self.rf.abstheta = steer + self.theta
        self.lb.abstheta = self.theta
        self.rb.abstheta = self.theta
        for wh in [self.lf,self.rf,self.lb,self.rb]:
            wh.abs_pos = self.pos + np.dot(Rot(self.theta),wh.rel_pos)
    


        
        


def main():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)   # 画面の大きさを設定する
    pygame.display.set_caption('Trajectory')   # 画面のタイトルを設定する
    font = pygame.font.Font(None, 60)

    #車を作成
    start_pos = [0.0,0.0]
    car1 = Car(np.array(start_pos))
    lf_rec = []
    rf_rec = []
    lb_rec = []
    rb_rec = []

    # 速度
    car1.vel = 0
    vel_coef = 1.0

    def draw_polygon_not_filled(ps,color):
        ps_sc = []
        for p in ps:
            ps_sc.append(XY(p))
        pygame.draw.polygon(screen,color,ps_sc,width=2)

    def draw_point(p):
        pygame.draw.circle(screen,(0,0,255),(p[0]*DRAW_SCALE+SCREEN_SIZE[0]/2,SCREEN_SIZE[1]-OFFSET-p[1]*DRAW_SCALE),1)

    def draw_polygon_filled(ps,color):
        ps_sc = []
        for p in ps:
            ps_sc.append(XY(p))
        pygame.draw.polygon(screen,color,ps_sc)

    def draw_carbody(car):
        p1 = car.pos + np.dot(Rot(car.theta),np.array([car.wid/2,(car.length+car.wh_bs)/2]))
        p2 = car.pos + np.dot(Rot(car.theta),np.array([-car.wid/2,(car.length+car.wh_bs)/2]))
        p3 = car.pos + np.dot(Rot(car.theta),np.array([-car.wid/2,(-car.length+car.wh_bs)/2]))
        p4 = car.pos + np.dot(Rot(car.theta),np.array([car.wid/2,(-car.length+car.wh_bs)/2]))
        ps = [p1,p2,p3,p4]
        if center_attach:
            for i in range(4):
                ps[i] = Trans(ps[i],car1.theta,car1.pos)
                
        draw_polygon_not_filled(ps,(0,200,0))
        #print((screen,(255,255,0),Rect(p1[0]+SCREEN_SIZE[0]/2,SCREEN_SIZE[1]-OFFSET-p1[1],p2[0]+SCREEN_SIZE[0]/2,SCREEN_SIZE[1]-OFFSET-p2[1]),2))
    
    
    def draw_wheel(wh):
        p = wh.abs_pos
        #本当はwheel_len/2だが、あえて2倍にしてみやすくしている
        p1 = p-vec(wh.abstheta)*(wheel_len)
        p2 = p+vec(wh.abstheta)*(wheel_len)
        if center_attach:
            p1 = Trans(p1,car1.theta,car1.pos)
            p2 = Trans(p2,car1.theta,car1.pos)
        p1*=DRAW_SCALE
        p2*=DRAW_SCALE
        pygame.draw.line(screen,(255,0,0),(p1[0]+SCREEN_SIZE[0]/2,SCREEN_SIZE[1]-OFFSET-p1[1]),(p2[0]+SCREEN_SIZE[0]/2,SCREEN_SIZE[1]-OFFSET-p2[1]),3)


    def draw_road(start_pos,Ls,wid,num,thetas):
        #長さLs,角度thetasの曲がった道
        p0 = np.array(start_pos)
        linepoints = []
        for i in range(len(Ls)):
            dir = vec(thetas[i])
            vdir = np.dot(Rot(math.pi/2),dir)
            
            p1 = p0+dir*Ls[i]
            p2 = p1+wid/2*vdir
            p3 = p2-(Ls[i]+wid/2)*dir
            p4 = p3-wid*vdir
            p5 = p1-wid/2*vdir
            ps = [p2,p3,p4,p5]
            if center_attach:
                p0 = Trans(p0,car1.theta,car1.pos)
                for i in range(4):
                    ps[i] = Trans(ps[i],car1.theta,car1.pos)
            draw_polygon_filled(ps,(120,120,120))
            linepoints.append(list(p0))
            p0 = p1
            if center_attach:
                p1 = Trans(p1,car1.theta,car1.pos)
        if center_attach:
            p0 = Trans(p0,car1.theta,car1.pos)
        linepoints.append(list(p0))
        for i in range(len(linepoints)):
            linepoints[i] = XY(linepoints[i])

        #白線を引く
        if num==2:
            pygame.draw.lines(screen,(255,255,255),False,linepoints,width=3)
            #print(linepoints)


            

            

    image = pygame.transform.rotozoom(pygame.image.load("handle.png").convert(), 0, 0.2)
    handle_diam = image.get_width()
    def draw_handle(steer):
        rotated_image = pygame.transform.rotozoom(image,math.degrees(steer*15),1)
        img_w = rotated_image.get_width()
        img_h = rotated_image.get_height()
        pygame.draw.circle(screen,(180,255,0),[(SCREEN_SIZE[0])/2,SCREEN_SIZE[1]-(OFFSET)/2],handle_diam/1.5)
        screen.blit(rotated_image,[(SCREEN_SIZE[0]-img_w)/2,SCREEN_SIZE[1]-(OFFSET+img_h)/2])
        
    steers = []
    def draw_graph():
        #ステアの履歴を描き出したい　一旦後？
        return
    while True:
        screen.fill((255, 255, 255))
        #ギアとブレーキの操作
        pressed_key = pygame.key.get_pressed()
        if pressed_key[K_w]:
            #ドライブギア
            vel_coef = 1.0
        if pressed_key[K_s]:
            #バックギア
            vel_coef = -1.0
        if pressed_key[K_LSHIFT]:
            #ブレーキ
            #一般的なブレーキは最大で0.6G程度だが、0.4G程度を超えると急ブレーキと感じられやすい
            break_acc = 9.8*0.4
            if car1.vel >= 0.0:
                car1.vel = max(car1.vel-break_acc*dt,0.0)
            else:
                car1.vel = min(car1.vel+break_acc*dt,0.0)
        if pressed_key[K_r]:
            #リセット
            car1 = Car(np.array(start_pos))
            lf_rec = []
            rf_rec = []
            lb_rec = []
            rb_rec = []  
                
        # mouseXをステアとして、mouseYをアクセルとする
        mouseX, mouseY = pygame.mouse.get_pos()
        #ステア
        steer = -(mouseX-SCREEN_SIZE[0]/2)/STEER_RATE
        steer = max(steer,-math.pi/5)
        steer = min(steer,math.pi/5)
        #ステア表示
        steer_disp = font.render(f'steer: {int(math.degrees(steer))} [deg]', True, (0, 0, 0))

        #アクセル
        # 一般車の最大加速度は0.6G程度より、マウスが一番上に来たあたりで0.6Gにする
        # クリープがあるので0.15m/s^2程度はデフォルトでかけておく
        acc = max((SCREEN_SIZE[1]-OFFSET-mouseY),0)*(0.6*9.8/(SCREEN_SIZE[1]-OFFSET))+0.15
        car1.vel += vel_coef*acc*dt
        #アクセル表示
        vel_disp = font.render(f'vel: {int(car1.vel*3.6)} [km/h]', True, (0, 0, 0))

        #print(steer)
        #print(car1.pos)
        

        #車を動かす
        
        car1.step_move(steer)


        #道の描画

        ## 左折
        [start_pos_lc,Ls_lc,thetas_lc]=[[start_pos[0]+1.9,0],[10,20],[0,math.pi/2]]
        #draw_road(start_pos_lc,Ls_lc,3.5,1,thetas_lc)

        ##右折
        [start_pos_rc,Ls_rc,thetas_rc]=[[start_pos[0]+1.5,0],[10,20],[0,-math.pi/2]]
        #draw_road(start_pos_rc,Ls_rc,3.5,1,thetas_rc)

        ## カーブは..まあなんとかなるかな

        ##S字クランク
        [start_pos_sc,Ls_sc,thetas_sc]=[[start_pos[0],0],[10,10,10],[0,math.pi/2,0]]
        #draw_road(start_pos_sc,Ls_sc,3.5,1,thetas_sc)

        ##S字カーブ
        [start_pos_scv,Ls_scv,thetas_scv]=[[start_pos[0],0],[10]+[0.25]*80+[10],[0]+list(np.linspace(0,math.pi*0.75,40))+list(np.linspace(math.pi*0.75,0,40))+[0]]
        #draw_road(start_pos_scv,Ls_scv,3.5,1,thetas_scv)

        ##複合経路も作れる
        draw_road(start_pos_sc,Ls_sc+Ls_scv,4,1,thetas_sc+thetas_scv)
        # 車輪の軌跡の描画
        for rec in [lf_rec,rf_rec,lb_rec,rb_rec]:
            for r in rec:
                if center_attach:
                    r = Trans(r,car1.theta,car1.pos)
                draw_point(r)

        # 車の描画
        draw_carbody(car1)
        for wh in [car1.lf,car1.rf,car1.lb,car1.rb]:
            draw_wheel(wh)
        lf_rec.append(car1.lf.abs_pos)
        rf_rec.append(car1.rf.abs_pos)
        lb_rec.append(car1.lb.abs_pos)
        rb_rec.append(car1.rb.abs_pos)
        
        #オフセットライン描画
        pygame.draw.line(screen,(0,0,0),[0,SCREEN_SIZE[1]-OFFSET],[SCREEN_SIZE[0],SCREEN_SIZE[1]-OFFSET],width=2)

        #ハンドルを描画
        draw_handle(steer)
        #テキスト描画
        screen.blit(steer_disp, [0,SCREEN_SIZE[1]-(OFFSET)/2])
        screen.blit(vel_disp,[SCREEN_SIZE[0]-300,SCREEN_SIZE[1]-OFFSET/2])
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            """
            # keydownイベントだと長押し対応できない？
            if event.type == KEYDOWN:
                print(event.key)
                if event.key == 119:
                    #ドライブギア
                    vel_coef = 1.0
                if event.key == 115:
                    #バックギア
                    vel_coef = -1.0
                if event.key == 1073742049:
                    # ブレーキ
                    if car1.vel >= 0.0:
                        car1.vel = max(car1.vel-7.0*dt,0.0)
                    else:
                        car1.vel = min(car1.vel+7.0*dt,0.0)
            """
        #更新してsleep
        pygame.display.update()
        time.sleep(dt)


if __name__ == '__main__':
    main()