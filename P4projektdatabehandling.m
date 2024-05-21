%% plotter crazyflie path

MYarr = crazyflieballast1;

% Plot af trajectory for det målte
X = MYarr.VarName6;
Y = MYarr.VarName7;
Z = MYarr.VarName8;

% Plot af trajectory for det målte
X2 = MYarr.VarName12;
Y2 = MYarr.VarName13;
Z2 = MYarr.VarName14;

plot3(X,Y,Z, 'b-',X2,Y2,Z2, 'r-')

axis([-500 1200 -500 1200 0 1500])
grid on

%% Plotter target og vejen vi tager derhen

MYarr = crazyflieLandtarg;

% Start 
Xstart = MYarr.VarName6(1:2);
Ystart = MYarr.VarName7(1:2);
Zstart = MYarr.VarName8(1:2);

% Slut kunne nok gøres bedre med en length of array funktion
Xslut = MYarr.VarName6(7182:7183);
Yslut = MYarr.VarName7(7182:7183);
Zslut = MYarr.VarName8(7182:7183);

% Plot af trajectory for det målte
X = MYarr.VarName6;
Y = MYarr.VarName7;
Z = MYarr.VarName8;

% Plot af trajectory for det målte
X2 = MYarr.VarName12;
Y2 = MYarr.VarName13;
Z2 = MYarr.VarName14;

% Plot grafer
plot3(X,Y,Z, 'b-',X2,Y2,Z2, 'r*-')
hold on;

% Plot start
plot3(Xstart,Ystart,Zstart,'b*')
plot3(Xslut,Yslut,Zslut,'bdiamond')

% Plot af trajectory for ref
% Definer tabel med reference værdier
Xreference = [8.1667;8.1667;1.037022095000000e+03;1.037022095000000e+03];
Yreference = [1.7358;1.7358;3.178472900000000e+02;3.178472900000000e+02];
Zreference = [60.9766;1000;1000;56.054413000000000];
RefValue = table(Xreference,Yreference,Zreference);

Xref = RefValue.Xreference;
Yref = RefValue.Yreference;
Zref = RefValue.Zreference;

% Plot ref
plot3(Xref, Yref, Zref, "g")


axis([-500 1200 -500 1200 0 1500])
grid on

hold off;


%% tager lidt tid at køre færdigt, nok ikke optimeret alt for godt
% finder længde fra path til ref, og plotter histogram for målinger, samt
% regner procentvis hvor meget af tiden vi er over max værdien 100.
% første sektion

sumdist = 0;
startid=1;
dt=2000;
hist=zeros(10000);
MYAR=crazyflie4pboxverttimed;
overhundrede = 0;
underhundrede = 0;

for i = startid:(startid+dt)
    v1 = [0,0,0];
    v2 = [0,0,1000];
    pt = [MYAR.VarName6(i),MYAR.VarName7(i),MYAR.VarName8(i)];

    distance = point_to_line(pt,v1,v2);
    sumdist = sumdist +  distance;
    hist(i)=distance;
    if distance < 100
        underhundrede = underhundrede + 1;
    end
    if distance > 100
        overhundrede = overhundrede + 1;
    end
end

mean1 = sumdist/dt



%
% anden sektion

sumdist = 0;
startid=startid + dt;

for i = startid:(startid + dt)
    v1 = [0,0,1000];
    v2 = [1000,0,1000];
    pt = [MYAR.VarName6(i),MYAR.VarName7(i),MYAR.VarName8(i)];

    distance = point_to_line(pt,v1,v2);
    sumdist = sumdist +  distance;
    hist(i)=distance;
    if distance < 100
        underhundrede = underhundrede + 1;
    end
    if distance > 100
        overhundrede = overhundrede + 1;
    end
end

mean2 = sumdist/2000

%
% tredje sektion

sumdist = 0;
startid=startid + dt;

for i = startid:(startid + dt)
    v1 = [1000,0,1000];
    v2 = [1000,0,1700];
    pt = [MYAR.VarName6(i),MYAR.VarName7(i),MYAR.VarName8(i)];

    distance = point_to_line(pt,v1,v2);
    sumdist = sumdist +  distance;
    hist(i)=distance;
    if distance < 100
        underhundrede = underhundrede + 1;
    end
    if distance > 100
        overhundrede = overhundrede + 1;
    end
end

mean3 = sumdist/2000
startid=startid + dt
%
% fjerde sektion

sumdist = 0;

for i = startid:(startid + dt)
    v1 = [1000,0,1700];
    v2 = [0,0,1700];
    pt = [MYAR.VarName6(i),MYAR.VarName7(i),MYAR.VarName8(i)];

    distance = point_to_line(pt,v1,v2);
    sumdist = sumdist +  distance;
    hist(i)=distance;
    if distance < 100
        underhundrede = underhundrede + 1;
    end
    if distance > 100
        overhundrede = overhundrede + 1;
    end
end

mean4 = sumdist/2000
startid=startid + dt
%
% femte sektion

sumdist = 0;

for i = startid:(startid + dt)
    v1 = [0,0,1700];
    v2 = [0,0,1000];
    pt = [MYAR.VarName6(i),MYAR.VarName7(i),MYAR.VarName8(i)];

    distance = point_to_line(pt,v1,v2);
    sumdist = sumdist +  distance;
    hist(i)=distance;
    if distance < 100
        underhundrede = underhundrede + 1;
    end
    if distance > 100
        overhundrede = overhundrede + 1;
    end
end

mean5 = sumdist/2000
overhundrede;
underhundrede;
sumafmaalinger = overhundrede + underhundrede;
procent = overhundrede/(sumafmaalinger/100) % dont think about it

plot(hist)

meanmean = (mean1 + mean2 + mean3 + mean4 + mean5) / 5

%% Plotter reference og målt path
MYAR=crazyflie4pboxverttimed;

% Plot af trajectory for det målte
X = MYAR.VarName6;
Y = MYAR.VarName7;
Z = MYAR.VarName8;


plot3(X,Y,Z)

axis([-500 1200 -500 1200 0 2000])
grid on

hold on;

Xreference = [0;0;1000;1000;0;0];
Yreference = [0;0;0;0;0;0];
Zreference = [0;1000;1000;1700;1700;1000];
RefValue = table(Xreference,Yreference,Zreference);

Xref = RefValue.Xreference;
Yref = RefValue.Yreference;
Zref = RefValue.Zreference;

% Plot ref
plot3(Xref, Yref, Zref, "g")

hold off;

%% Plotter for uden ballast x, y og z på hver deres plot ud fra tid

Xreference = [0;0;1000;1000;0;0];
Yreference = [0;0;0;0;0;0];
Zreference = [1500;1500;1000;1000;500;500];
TreferenceX = [0;2200;2200;4200;4200;12000];
TreferenceY = [0;2200;2200;4200;4200;12000];
TreferenceZ = [0;6200;6200;8200;8200;12000];


MYarr = crazyflieballastnone;
MYarr1 = crazyflieballastnone1;
MYarr2 = crazyflieballastnone2;
% igen 6,7,8 som x,y,z


nexttile
plot(MYarr.VarName1,MYarr.VarName6)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName6)
plot(MYarr2.VarName1,MYarr2.VarName6)
plot(TreferenceX,Xreference)
legend('Test 1','Test 2','Test 3','Ref')
axis([0 12000 -500 1200])
title('X axis')
hold off;

nexttile
plot(MYarr.VarName1,MYarr.VarName7)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName7)
plot(MYarr2.VarName1,MYarr2.VarName7)
plot(TreferenceY,Yreference)
legend('Test 1','Test 2','Test 3','Ref')
%axis([-500 1200 0 12000])
axis([0 12000 -500 1200])
title('Y axis')
hold off;

nexttile
plot(MYarr.VarName1,MYarr.VarName8)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName8)
plot(MYarr2.VarName1,MYarr2.VarName8)
plot(TreferenceZ,Zreference)
legend('Test 1','Test 2','Test 3','Ref')
axis([0 12000 0 1700])
title('Z axis')
hold off;

sumdistX = 0;
for i = 200:2200
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 2200:4200
    pointx1 = 1000;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 4200:6856
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 6859:10200
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
meanX = sumdistX/10000 

sumdistY = 0;
for i = 200:6856
    pointy1 = 0;
    point11 = MYarr.VarName7(i);
    point12 = MYarr1.VarName7(i);
    point13 = MYarr2.VarName7(i);
    distanceY1 = point11 - pointy1;
    distanceY2 = point12 - pointy1;
    distanceY3 = point13 - pointy1;
    sumdistY = sumdistY + distanceY1;
    sumdistY = sumdistY + distanceY2;
    sumdistY = sumdistY + distanceY3;
end
for i = 6859:10200
    pointy1 = 0;
    point11 = MYarr.VarName7(i);
    point12 = MYarr1.VarName7(i);
    point13 = MYarr2.VarName7(i);
    distanceY1 = point11 - pointy1;
    distanceY2 = point12 - pointy1;
    distanceY3 = point13 - pointy1;
    sumdistY = sumdistY + distanceY1;
    sumdistY = sumdistY + distanceY2;
    sumdistY = sumdistY + distanceY3;
end
meanY = sumdistY/10000

sumdistZ = 0;
for i = 200:6200
    pointz1 = 1500;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 6200:6856
    pointz1 = 1000;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 6859:8200
    pointz1 = 1000;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 8200:10200
    pointz1 = 500;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
meanZ = sumdistZ/6000

meanmean = (meanX+meanY+meanZ)/3

%% Plotter for med ballast x, y og z på hver deres plot ud fra tid

Xreference = [0;0;1000;1000;0;0];
Yreference = [0;0;0;0;0;0];
Zreference = [1500;1500;1000;1000;500;500];
TreferenceX = [0;2200;2200;4200;4200;12000];
TreferenceY = [0;2200;2200;4200;4200;12000];
TreferenceZ = [0;6200;6200;8200;8200;12000];


MYarr = crazyflieballast;
MYarr1 = crazyflieballast1;
MYarr2 = crazyflieballast2;
% igen 6,7,8 som x,y,z


nexttile
plot(MYarr.VarName1,MYarr.VarName6)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName6)
plot(MYarr2.VarName1,MYarr2.VarName6)
plot(TreferenceX,Xreference)
legend('Test 1','Test 2','Test 3','Ref')
axis([0 12000 -500 1200])
title('X axis')
hold off;

nexttile
plot(MYarr.VarName1,MYarr.VarName7)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName7)
plot(MYarr2.VarName1,MYarr2.VarName7)
plot(TreferenceY,Yreference)
legend('Test 1','Test 2','Test 3','Ref')
%axis([-500 1200 0 12000])
axis([0 12000 -500 1200])
title('Y axis')
hold off;

nexttile
plot(MYarr.VarName1,MYarr.VarName8)
hold on;
plot(MYarr1.VarName1,MYarr1.VarName8)
plot(MYarr2.VarName1,MYarr2.VarName8)
plot(TreferenceZ,Zreference)
legend('Test 1','Test 2','Test 3','Ref')
axis([0 12000 0 1700])
title('Z axis')
hold off;

sumdistX = 0;
for i = 200:2200
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 2200:4200
    pointx1 = 1000;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 4200:6856
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
for i = 6859:10200
    pointx1 = 0;
    point11 = MYarr.VarName6(i);
    point12 = MYarr1.VarName6(i);
    point13 = MYarr2.VarName6(i);
    distanceX1 = point11 - pointx1;
    distanceX2 = point12 - pointx1;
    distanceX3 = point13 - pointx1;
    sumdistX = sumdistX + distanceX1;
    sumdistX = sumdistX + distanceX2;
    sumdistX = sumdistX + distanceX3;
end
meanX = sumdistX/10000 

sumdistY = 0;
for i = 200:6856
    pointy1 = 0;
    point11 = MYarr.VarName7(i);
    point12 = MYarr1.VarName7(i);
    point13 = MYarr2.VarName7(i);
    distanceY1 = point11 - pointy1;
    distanceY2 = point12 - pointy1;
    distanceY3 = point13 - pointy1;
    sumdistY = sumdistY + distanceY1;
    sumdistY = sumdistY + distanceY2;
    sumdistY = sumdistY + distanceY3;
end
for i = 6859:10200
    pointy1 = 0;
    point11 = MYarr.VarName7(i);
    point12 = MYarr1.VarName7(i);
    point13 = MYarr2.VarName7(i);
    distanceY1 = point11 - pointy1;
    distanceY2 = point12 - pointy1;
    distanceY3 = point13 - pointy1;
    sumdistY = sumdistY + distanceY1;
    sumdistY = sumdistY + distanceY2;
    sumdistY = sumdistY + distanceY3;
end
meanY = sumdistY/10000

sumdistZ = 0;
for i = 200:6200
    pointz1 = 1500;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 6200:6856
    pointz1 = 1000;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 6859:8200
    pointz1 = 1000;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
for i = 8200:10200
    pointz1 = 500;
    point11 = MYarr.VarName8(i);
    point12 = MYarr1.VarName8(i);
    point13 = MYarr2.VarName8(i);
    distanceZ1 = point11 - pointz1;
    distanceZ2 = point12 - pointz1;
    distanceZ3 = point13 - pointz1;
    sumdistZ = sumdistZ + distanceZ1;
    sumdistZ = sumdistZ + distanceZ2;
    sumdistZ = sumdistZ + distanceZ3;
end
meanZ = sumdistZ/6000

meanmean = (meanX+meanY+meanZ)/3


%% Funktion til at regne længde fra punkt til linje
% functions


function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      b = pt - v2;
      d = norm(cross(a,b)) / norm(a);
end
