function K = get_k_length(leg_length)
   
    %theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
    %x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
    %phi   : 机体与水平夹角                LM  : 摆杆重心到其转轴距离
    %T     ：驱动轮输出力矩                 l   : 机体重心到其转轴距离
    %Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
    %N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量

    syms theta(t) x(t) phi(t) T Tp R L LM l mw mp M Iw Ip IM g  
    syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

    R1=0.075;                         %驱动轮半径
    L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
    LM1=leg_length/2;                 %摆杆重心到其转轴距离
    l1=0.03;                          %机体质心距离转轴距离
    mw1=1.18;                         %驱动轮质量
    mp1=1.078;                         %杆质量
    M1=5.486;                          %机体质量
 %   Iw1 = 0.000120085216;              %驱动轮转动惯量
 %   Ip1 = 0.04797207231;               %摆杆转动惯量
  %  IM1 = 0.20703226313;                %机体绕质心转动惯量

   Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=mp1*((L1+LM1)^2+0.05^2)/12.0; %摆杆转动惯量
   IM1=M1*(0.3^2+0.12^2)/12.0;       %机体绕质心转动惯量

    
    NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
    N = NM + mp*diff(x + L*sin(theta),t,2);
    PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
    P = PM +mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
    eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);
    
    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

    
    [f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);

    A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    A=double(A);
    B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    B=double(B);
    
  %  Q = diag([100 1 500 100 5000 1]);%theta d_theta x d_x phi d_phi
  %  R = [240 0;0 25];                %T Tp

  %  Q = diag([150 880 5800 250 80000 500]);%theta d_theta x d_x phi d_phi
  %  Q = diag([1 1 80 150 4500 5]);
  %  Q = diag([1 1 80 150 4500 5]);
  %  Q = diag([15 4 80 150 4500 5]); % 轮子有B动静
  %  Q = diag([50 5 80 150 4500 5]); % 整体效果不错 但有时腿部会震
  %  Q = diag([50 1 80 150 4500 5]);
  %  Q = diag([1 0.65 80 150 4500 1]);

  % 板凳模型参数
  %  Q = diag([20 1 1 1 3000 1]); % 初步平衡
  %  Q = diag([20 1 1 1 3500 1]);
  %  Q = diag([20 1 10 1 3500 1]); % 腿震
  %  Q = diag([20 1 1 10 3500 1]); % 同上
  %  Q = diag([20 5 1 10 3500 1]); % 轮子功耗惩罚为5 不震 但会往一个方向走
  %  Q = diag([20 5 1 70 3500 1]); % 轮子功耗惩罚为6 略震
  %  Q = diag([20 5 1 70 4000 1]); % 发现底盘俯仰角很软
  %  Q = diag([10 3 3 70 4000 10]); % 腿爆震
  %  Q = diag([10 3 3 34 4000 10]); % 搭配5 4 效果比较好 DELAUT L0 : 0.22f
  %  在0.2不适合 微调R

  % R:4.4 4 之后开始上轮腿模型:
  %  Q = diag([100 50 3 34 4000 10]); % 发散
  %  Q = diag([100 40 3 34 4000 10]); % 同上
  %  Q = diag([100 20 3 34 4000 10]);
  %  Q = diag([50 50 3 34 4000 10]);  % 小抖
  %  Q = diag([28 9 3 34 4000 10]); % 0.16腿长效果不错 18不行
  %  Q = diag([100 10 3 34 4500 1]);
  %  Q = diag([100 10 3 34 4700 1]); % 3.
  %  Q = diag([100 10 3 34 5000 1]);
  %  Q = diag([100 7 3 34 5000 1]); % phi有点超调
  %  Q = diag([100 7 3 34 4950 1]); % 偶尔有一丢丢小震
  %  Q = diag([100 7 3 34 4920 1]); % phi_dot震荡 需要其他地方去抵抗它
  %  Q = diag([103 7 3 34 4920 1]); % theta_dot震荡
  %  Q = diag([103 6 3 34 4920 1]); % phi_dot震荡略大 phi theta_dot小震
  %  Q = diag([110 5.5 3 34 4920 1]); % theta_dot震荡
  %  Q = diag([150 7 3 34 4920 1]); % theta收敛不错 theta_dot震荡
  %  Q = diag([200 7 3 34 4920 1]); % theta_dot震荡
  %  Q = diag([500 1 3 34 6000 1]);

  %  Q = diag([28 9 3 34 4000 10]);
  %  Q = diag([28 9 3 34 4000 1]);
  %  Q = diag([28 9 3 34 19000 1]); % 不错 大力下压腿、侧踢腿的时候腿震动较大
  %  Q = diag([28 9 3 34 18500 1]);
  %  Q = diag([28 9 3 34 19000 1]); % 腿长长的时候会一直往一个方向走  疑似theta
  %  Q = diag([28 9 3 34 30000 1]); % 这一套搭配theta : 0.0348 phi : 0 变腿长停住的效果不错
  %  Q = diag([28 9 3 34 70000 1]); % 震荡
  %  Q = diag([28 9 3 34 35000 1]); % 不震了
  %  Q = diag([28 9 3 40 35000 1]); % 一直会往前拱 应该是速度环超调了
  %  Q = diag([28 9 5 50 35000 1]); % 起得有点猛
  %  Q = diag([27 9 10 50 35000 1]); % 比较正常 溜车不是很严重 腿耐踢还行

  % 4 1
%    Q = diag([5 9 10 50 25000 1]); % 机体有点小晃 位移还需要调
%    Q = diag([5 9 10 50 23000 1]);
     Q = diag([9 5 10 50 18000 1]);

  %  R = [60 0;0 40];                %T Tp
  %  R = [5 0;0 4];
  %  R = [4.8 0;0 4];
     R = [4 0;0 1];
    
    K = lqr(A,B,Q,R); % lqr函数返回一个 p x n 的矩阵
  
end