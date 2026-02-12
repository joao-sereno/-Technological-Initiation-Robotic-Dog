%Definindo as matrizes de transformações homogêneas

L1 = 20.75/100;
L2 = 33.75/100;
L3 = -28.5/100;
L4 = -88.4/100;
L5 = -68.4/100;

[e1_afrente,e1_atras,e2_afrente,e2_atras] = creatHomoMatrix(L1,L2,L3,L4,L5);
%%
%Transformando as matrizes em corpos rigídos com formato/estrura de
%árvores.
[leg1_afrente,leg1_atras,leg2_afrente,leg2_atras] = transformHomoMatrixInTree(e1_afrente,e1_atras,e2_afrente,e2_atras);
%%
%se3(leg1_afrente.getTransform([0 0 0],"link8")).trvec
%se3(leg1_atras.getTransform([0 0 0],"link8")).trvec
%%

leg1_afrente.show();hold on
leg1_atras.show();
leg2_afrente.show();
leg2_atras.show();hold off
% leg1_afrente.show([deg2rad(-10) deg2rad(45) deg2rad(-45)])
% hold on
% leg1_atras.show([deg2rad(-10) deg2rad(45) deg2rad(-45)])
% hold on
% leg2_afrente.show([deg2rad(-10) deg2rad(45) deg2rad(-45)])
% hold on
% leg2_atras.show([deg2rad(-10) deg2rad(45) deg2rad(-45)])
% hold off
%%
%qcycle1 e qcycle2 (Andar para frente com a perna direita e esquerda)
xf1 = 40/100+33.75/100; xb1 = -40/100+33.75/100; y1 = -20.75/100; zu1 = -120/100; zd1 = -140/100;
via1 = [xf1 y1 zd1
    xb1 y1 zd1
    xb1 y1 zu1
    xf1 y1 zu1
    xf1 y1 zd1];
x1 = mstraj(via1,[],[3 0.25 0.5 0.25],[],0.01,0.1);

if(mod(length(x1),2)~=0)
    x1(length(x1),:) = [];
    length(x1)
end


xf2 = 40/100-33.75/100; xb2 = -40/100-33.75/100; y2 = 20.75/100; zu2 = -120/100; zd2 = -140/100;
via2 = [xf2 y2 zd2
    xb2 y2 zd2
    xb2 y2 zu2
    xf2 y2 zu2
    xf2 y2 zd2];
x2 = mstraj(via2,[],[3 0.25 0.5 0.25],[],0.01,0.1);

if(mod(length(x2),2)~=0)
    x2(length(x2),:) = [];
    length(x2)
end

global qcycle1_afrente;
global qcycle1_atras;

qcycle1_afrente = ikineTrajNum(leg1_afrente,se3(eye(3),x1),"link8", weights=[0 0 0 1 1 1]);
qcycle1_atras = ikineTrajNum(leg1_atras,se3(eye(3),x2),"link8", weights=[0 0 0 1 1 1]);

 plot3(x1(:,1),x1(:,2),x1(:,3));
 hold on
 for k = 1:length(x1) 
     if (x1(k,3) > zd1)
         disp(k)
         plot3(x1(k,1),x1(k,2),x1(k,3),'r*','MarkerSize',15);
         xlabel('X-label');
         ylabel('Y-label');
         zlabel('Z-label');
         grid on
         title('Point added when the third column changes value');
         hold off
         break
     end
end
%%
xf3 = 40/100+33.75/100; xb3 = -40/100+33.75/100; y3 = 20.75/100; zu3 = -120/100; zd3 = -140/100;
via3 = [xf3 y3 zd3
    xb3 y3 zd3
    xb3 y3 zu3
    xf3 y3 zu3
    xf3 y3 zd3];
x3 = mstraj(via3,[],[3 0.25 0.5 0.25],[],0.01,0.1);

if(mod(length(x3),2)~=0)
    x3(length(x3),:) = [];
    length(x3)
end

xf4 = 40/100-33.75/100; xb4 = -40/100-33.75/100; y4 = -20.75/100; zu4 = -120/100; zd4 = -140/100;
via4 = [xf4 y4 zd4
    xb4 y4 zd4
    xb4 y4 zu4
    xf4 y4 zu4
    xf4 y4 zd4];
x4 = mstraj(via4,[],[3 0.25 0.5 0.25],[],0.01,0.1);

if(mod(length(x4),2)~=0)
    x4(length(x4),:) = [];
    length(x4)
end

global qcycle2_afrente;
global qcycle2_atras;
qcycle2_afrente = ikineTrajNum(leg2_afrente,se3(eye(3),x3),"link8", weights=[0 0 0 1 1 1]);
qcycle2_atras = ikineTrajNum(leg2_atras,se3(eye(3),x4),"link8", weights=[0 0 0 1 1 1]);


%%
% global d
% d = rateControl(200);
% j = 1;
% while(1)
%      for i = 1:size(qcycle2_atras,1)
%          leg1_afrente.show(gait(qcycle1_afrente,i,0,false),FastUpdate=true,PreservePlot=false);hold on
%          leg2_afrente.show(gait(qcycle2_afrente,i,100,false),FastUpdate=true,PreservePlot=false);
%          leg1_atras.show(gait(qcycle1_atras,i,200,true),FastUpdate=true, PreservePlot= false);
%          leg2_atras.show(gait(qcycle2_atras,i,300,true),FastUpdate=true,PreservePlot=false);
%          hold off
%          d.waitfor;
%      end
%     j = j + 1;
%     if (j >= 1)
%         break;
%     end
% end
%%
offsetFirstLeg = k;
offsetSecondLeg = round(k + (3*length(x4))/4);
offsetThirdLeg = round(k + length(x2)/2);
offsetFourthLeg = round(k + length(x3)/4);
global r
r = rateControl(200);

while(1)
    for i = i:size(qcycle2_atras,1)
        gait1 = mod(i + offsetFirstLeg-1,length(qcycle1_afrente))+1;
        gait2 = mod(i + offsetSecondLeg-1,length(qcycle2_afrente))+1;
        gait3 = mod(i + offsetThirdLeg-1,length(qcycle1_atras))+1;
        gait4 = mod(i + offsetFourthLeg-1,length(qcycle2_atras))+1;
        ang1 = rad2deg(qcycle1_afrente(gait1,:));
        ang2 = rad2deg(qcycle2_afrente(gait2,:));
        ang3 = rad2deg(qcycle1_atras(gait3,:));
        ang4 = rad2deg(qcycle2_atras(gait4,:));
        fprintf("\n ang1_q1: %.4f \n ang1_q2: %.4f \n ang1_q3: %.4f" + ...
            "\n ang2_q1: %.4f\n ang2_q2: %.4f\n ang2_q3: %.4f\n ang3_q1: %.4f\n" + ...
            " ang3_q2: %.4f\n ang3_q3: %.4f\n ang4_q1: %.4f\n ang4_q2: %.4f\n ang4_q3: %.4f\n",ang1(1,1)...
        ,ang1(1,2),ang1(1,3),ang2(1,1),ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2) ...
        ,ang3(1,3),ang4(1,1),ang4(1,2),ang4(1,3));
        r.waitfor;
    end
    j = j + 1;
    if (j >= 1)
        break;
    end
end
global d
d = rateControl(200);
j = 1;
while(1)
     for i = 1:size(qcycle2_atras,1)
         leg1_afrente.show(gait(qcycle1_afrente,i,offsetFirstLeg,false),FastUpdate=true,PreservePlot=false);hold on
         leg2_afrente.show(gait(qcycle2_afrente,i,offsetSecondLeg,false),FastUpdate=true,PreservePlot=false);
         leg1_atras.show(gait(qcycle1_atras,i,offsetThirdLeg,false),FastUpdate=true, PreservePlot= false);
         leg2_atras.show(gait(qcycle2_atras,i,offsetFourthLeg,false),FastUpdate=true,PreservePlot=false);
         hold off
         d.waitfor;
     end
    j = j + 1;
    if (j > 3)
        break;
    end
end
%% 
function [e1,e2,e3,e4] = creatHomoMatrix(L1,L2,L3,L4,L5)
    e1 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e2 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e3 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e4 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
end

%%
function [l1,l2,l3,l4] = transformHomoMatrixInTree(e1,e2,e3,e4)
    l1 = ets2rbt(e1);
    l2 = ets2rbt(e2);
    l3 = ets2rbt(e3);
    l4 = ets2rbt(e4);
end