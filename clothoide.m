dt = 0.01 ;
t=(-2*pi):dt:(2*pi) ;
x = cumtrapz(sin(t.^2)) * dt ;
y = cumtrapz(cos(t.^2)) * dt ;

figure
hold on
axis equal
grid on
plot(x,y,'b-') ;
