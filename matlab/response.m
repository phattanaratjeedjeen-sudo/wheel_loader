num = 1;
den = [1 1];
sys = tf(num, den);
S = stepinfo(sys);

[y,t] = step(sys);
ts = S.SettlingTime;
% find index closest to settling time (use last index <= ts)
idx = find(t <= ts, 1, 'last');
if isempty(idx) || idx==length(t)
    % fallback: use last two points
    idx = max(1,length(t)-1);
end
% compute slope using small window after idx (use next point)
dt = t(idx+1) - t(idx);
slope_at_ts = (y(idx+1) - y(idx)) / dt;

% plot response and slope vs time
figure;
yyaxis left
plot(t,y,'b-','LineWidth',1.5)
ylabel('Step Response y(t)')
hold on
% mark settling time and point used for slope
plot(t(idx),y(idx),'ro','MarkerFaceColor','r')
xline(ts,'r--','Settling Time','LabelHorizontalAlignment','left')

yyaxis right
% compute instantaneous slope (finite difference) for plotting
dy = diff(y)./diff(t);
t_mid = t(1:end-1) + diff(t)/2;
plot(t_mid,dy,'g-','LineWidth',1)
ylabel('Slope dy/dt')
% mark slope at settling-time location
plot(t(idx),slope_at_ts,'ms','MarkerFaceColor','m')

xlabel('Time (s)')
title('Step Response and Slope vs Time')
legend('y(t)','point used for slope','dy/dt','slope@ts','Location','best')
grid on
hold off

S
slope_at_ts