% test script
close all;
clear all;
clc;
X = [
-177.481
-180.103
-181.936
-183.939
-183.12
-180.331
-181.304
-179.734
-176.79
-172.538
];

Y = [
101.825
116.382
140.898
163.578
167.251
162.887
137.344
114.345
94.411
94.087
];

Z = [
-21.042
-10.231
-4.915
-8.711
-29.916
-56.7
-63.282
-65.254
-48.308
-27.986
];

index1 = [
 9 0 1
 2 3 4
 4 5 6
 6 7 8
 8 9 1
 1 2 4
 4 6 8
 8 1 4
];
X2 = [
-180.166
-190.065
-191.909
-193.877
-193.082
-190.243
-191.255
-189.67
-186.752
-181.342
];

Y2 = [
107.252
115.625
140.208
162.608
166.494
163.559
136.403
113.594
93.6538
89.6885
];

Z2 = [
-29.0006
-10.6609
-5.17032
-9.26283
-30.3459
-57.8416
-62.9802
-64.4107
-48.7379
-26.214
];

X3 = [
-174.796
-170.141
-171.963
-174.001
-173.158
-170.419
-171.353
-169.798
-166.828
-163.734
];

Y3 = [
96.3979
117.139
141.588
164.548
168.008
162.215
138.285
115.096
95.1682
98.4855
];

Z3 = [
-13.0834
-9.80113
-4.65968
-8.15917
-29.4861
-55.5584
-63.5838
-66.0973
-47.8781
-29.758
];

X4 = [
-180.166
-174.796
-190.065
-170.141
-191.909
-171.963
-193.877
-174.001
-193.082
-173.158
-190.243
-170.419
-191.255
-171.353
-189.67
-169.798
-186.752
-166.828
-181.342
-163.734
];

Y4 = [
107.252
96.3979
115.625
117.139
140.208
141.588
162.608
164.548
166.494
168.008
163.559
162.215
136.403
138.285
113.594
115.096
93.6538
95.1682
89.6885
98.4855
];

Z4 = [
-29.0006
-13.0834
-10.6609
-9.80113
-5.17032
-4.65968
-9.26283
-8.15917
-30.3459
-29.4861
-57.8416
-55.5584
-62.9802
-63.5838
-64.4107
-66.0973
-48.7379
-47.8781
-26.214
-29.758
];

index2 = [
 0 1 2
 3 2 1
 2 3 4
 5 4 3
 4 5 6
 7 6 5
 6 7 8
 9 8 7
 8 9 10
 11 10 9
 10 11 12
 13 12 11
 12 13 14
 15 14 13
 14 15 16
 17 16 15
 16 17 18
 19 18 17
 18 19 1
 18 1 0
];



plot3(X,Y,Z, 'LineWidth',6,'Color','r');

xlabel("x [cm]");
ylabel("y [cm]");
zlabel("z [cm]");
grid on;
hold on;


%color = ['r','g','b','y'];
color = ['b'];
color_it = 1;
% for i = 1:length(index1(:,1))
%     
%     tri = [X(index1(i,1)+1) Y(index1(i,1)+1) Z(index1(i,1)+1);
%            X(index1(i,2)+1) Y(index1(i,2)+1) Z(index1(i,2)+1);
%            X(index1(i,3)+1) Y(index1(i,3)+1) Z(index1(i,3)+1)];
%     
%     h = fill3(tri(:,1)',tri(:,2)', tri(:,3)',color(color_it));
%     alpha(h, 0.5);
% 
%     color_it = color_it + 1;
%     if (color_it > length(color))
%         color_it = 1;
%     end
% 
% end

color_it = 1;
for i = 1:length(index1(:,1))
    
    tri = [X2(index1(i,1)+1) Y2(index1(i,1)+1) Z2(index1(i,1)+1);
           X2(index1(i,2)+1) Y2(index1(i,2)+1) Z2(index1(i,2)+1);
           X2(index1(i,3)+1) Y2(index1(i,3)+1) Z2(index1(i,3)+1)];
    
    h = fill3(tri(:,1)',tri(:,2)', tri(:,3)',color(color_it));
    alpha(h, 0.5);

    color_it = color_it + 1;
    if (color_it > length(color))
        color_it = 1;
    end

end

color_it = 1;
for i = 1:length(index1(:,1))
    
    tri = [X3(index1(i,1)+1) Y3(index1(i,1)+1) Z3(index1(i,1)+1);
           X3(index1(i,2)+1) Y3(index1(i,2)+1) Z3(index1(i,2)+1);
           X3(index1(i,3)+1) Y3(index1(i,3)+1) Z3(index1(i,3)+1)];
    
    h = fill3(tri(:,1)',tri(:,2)', tri(:,3)',color(color_it));
    alpha(h, 0.5);

    color_it = color_it + 1;
    if (color_it > length(color))
        color_it = 1;
    end

end

color_it = 1;
for i = 1:length(index2(:,1))
    
    tri = [X4(index2(i,1)+1) Y4(index2(i,1)+1) Z4(index2(i,1)+1);
           X4(index2(i,2)+1) Y4(index2(i,2)+1) Z4(index2(i,2)+1);
           X4(index2(i,3)+1) Y4(index2(i,3)+1) Z4(index2(i,3)+1)];
    
    h = fill3(tri(:,1)',tri(:,2)', tri(:,3)','b');
    alpha(h, 0.1);

    color_it = color_it + 1;
    if (color_it > length(color))
        color_it = 1;
    end

end

plot3(-180,140,-30,'o','Color','g','LineWidth',6);

