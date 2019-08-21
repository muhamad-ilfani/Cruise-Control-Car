%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: BayesRuleMatlabRegression.m
% Demonstration code forBayes' Rule: A Tutorial Introduction to Bayesian Analysis
% JV Stone, 2012.
% Copyright: 2012, JV Stone,  Sheffield University, Sheffield, England.    
% The MatLab code below is version 7.5. This code can be downloaded from http://jim-stone.staff.shef.ac.uk/BayesBook/Matlab.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

s=6; % set random number seed.
rand('seed',s);	randn('seed',s);

% Make noisy data with slope m and intercept c.
% Define 10 salary values for horizontal axis.
s = 1:1:11; s=s';
% Set true values of slope m and intercept c.
m = 0.5;
c = 3;
% Set standard deviation of each measured height.
sds = 2*[1:11]'/10; %rand(size(s))*2;
sds = sds .* [1:11]'/10;
% Use sds to make vector of noise.
eta = randn(size(s)).*sds;
% Pretend some data points are extreme.
eta(8)=-1; eta(9)=-3; eta(11)=-3;
% Find observed values of x (with noise added).
x = m*s + c + eta;

% Weighted Least Squares regression. We could find the solution using the smallest value in the Farray below also.
% Find weightings w (discount) for each data point.
vars = sds.^2;
w=1./vars;
%w=ones(size(w)); % un-comment this line to get solution based on uniform noise terms.
ss=[ones(size(s)) s]; % prepend column of ones so that solution includes intercept term.
[params,stdx,mse,S] =lscov(ss,x,w);
mest2 = params(2);
cest2 = params(1);
%%%%%%%%

xest2 = mest2.*s + cest2;
c0 = cest2;
m0 = mest2;

% Plot fitted line xest (=xhat in text) and data points.
figure(1); clf;
plot(s,x,'k*',s,xest2, 'k', 'LineWidth',2, 'MarkerSize',10);
set(gca,'Linewidth',2); set(gca,'FontSize',20);
xlabel('Salary, {\it s} (groats)');
ylabel('Height, {\it x} (feet)');
set(gca,'XLim',[0 12],'FontName','Ariel');
set(gca,'YLim',[0 9],'FontName','Ariel');
% Plot sd  from each data point
for i=1:11
    x1 = x(i)-sds(i);  x2=x(i)+sds(i);
    s1 = i;  s2 = i;
    ss = [s1 s2];  xx = [x1 x2];
    hold on;
    plot(ss,xx,'k','LineWidth',2); hold off;
end

% Code for 2d plot
m = mest2;
mmin = mest2-1; % 0.5
mmax = mest2+1;

c = cest2;
cmin = cest2-1; %0.4
cmax = cest2+1;
minc = (mmax-mmin)/100;
cinc = (cmax-cmin)/100;
Fs = [];
ms = mmin:minc:mmax;
nm = length(ms);
cs = cmin:cinc:cmax;
nc = length(cs);

Farray = zeros(nm,nc);
for m1 = 1:nm 
    for c1 = 1:nc 
        mval=ms(m1);
        cval=cs(c1);
        y1 = mval*s + cval;
        F1 = ((x-y1)./sds).^2;
        Farray(m1,c1)=sum(F1);
        Fs = [Fs F1];
    end
end

figure(2);
Z1 = Farray';
zmin=min(Z1(:));
zmax=max(Z1(:));
range=zmax-zmin;
v=zmin:range/10:zmax;
% Adjust spacing of contour lines.
v=0:0.5:8; v=exp(v); v=v*range/max(v); 
[X Y]=meshgrid(ms,cs);
contour(X,Y,Z1,v,'LineWidth',2);
colormap([0 0 0]);
set(gca,'Linewidth',2);   set(gca,'FontSize',20);
xlabel('Slope, {\it m}');  ylabel('Intercept, {\it c}');

figure(3);
grid on;
set(gca,'LineWidth',2);
surfl(X,Y,Z1)
shading interp
set(gca,'Linewidth',2); set(gca,'FontSize',20);
xlabel('{\it m}');
ylabel('{\it c}');
zlabel('F');
view(-13,30);
colormap(gray);
set(gca,'XLim',[-0.5 1.5],'FontName','Ariel');
set(gca,'YLim',[2 4],'FontName','Ariel');

return
%%%%%%%%%%%%%%%%%%