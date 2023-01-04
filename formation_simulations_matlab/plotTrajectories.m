%% Gerono Leminscate
t = linspace(0,2*pi,1001)
x_lem = cos(t)
y_lem = cos(t).*sin(t)


%%  Squares
P = [-1  1 1 -1 -1
     -1 -1 1  1 -1
      2  2 2  2  2];

T = [0, 2, 4, 6, 8];

linear_square_ts = trajectory_interpolation(P,T,0.01,'linear');
poly_square_ts = trajectory_interpolation(P,T,0.01,'fifth');


%% Plotting
figure();
% Lem
subplot(3,2,1); grid on; hold on
plot(x_lem, y_lem)
subplot(3,2,2); grid on; hold on;
plot(t,x_lem)
plot(t,y_lem)
% LinSquare
subplot(3,2,3); grid on; hold on
plot(linear_square_ts.Data(:,1), linear_square_ts.Data(:,2))
subplot(3,2,4); grid on; hold on;
plot(linear_square_ts.Time,linear_square_ts.Data(:,1))
plot(linear_square_ts.Time,linear_square_ts.Data(:,2))
% PolySquare
subplot(3,2,5); grid on; hold on
plot(poly_square_ts.Data(:,1), poly_square_ts.Data(:,2))
subplot(3,2,6); grid on; hold on;
plot(poly_square_ts.Time, poly_square_ts.Data(:,1))
plot(poly_square_ts.Time, poly_square_ts.Data(:,2))