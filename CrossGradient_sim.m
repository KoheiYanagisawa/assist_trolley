% 台車のパラメータ
L = 1.0;                 % 重心から前輪までの距離 (m)
L_wheel = 0.8;           % 左右後輪の間隔 (m)
dt = 0.1;                % 時間刻み (秒)
T = 20;                  % シミュレーション時間 (秒)
alpha = deg2rad(0);      % 横断勾配の角度 (度)

% 初期状態
x = 0;                   % 台車の初期 x 座標 (m)
y = 0;                   % 台車の初期 y 座標 (m)
theta = pi / 2;          % 台車の初期向き (ラジアン)

% 制御入力（速度と角速度）
v = 1.0;                 % 台車の直進速度 (m/s)
omega = 0;             % 台車の角速度 (rad/s)

% 状態の記録用
x_history = zeros(T/dt, 1);
y_history = zeros(T/dt, 1);
theta_history = zeros(T/dt, 1);
x_front_history = zeros(T/dt, 1); % 前輪の x 座標
y_front_history = zeros(T/dt, 1); % 前輪の y 座標
v_left_history = zeros(T/dt, 1);
v_right_history = zeros(T/dt, 1);

% シミュレーションループ
for i = 1:(T/dt)
    % 状態の記録
    x_history(i) = x;
    y_history(i) = y;
    theta_history(i) = theta;

    % 前輪の位置を計算
    x_front = x + L * cos(theta);
    y_front = y + L * sin(theta);
    x_front_history(i) = x_front;
    y_front_history(i) = y_front;

    % 横断勾配による横方向速度の影響
    v_cross = v * tan(alpha);
    
    % 左右後輪の速度計算
    v_left = v - (L_wheel * omega) / 2;
    v_right = v + (L_wheel * omega) / 2;
    v_left_history(i) = v_left;
    v_right_history(i) = v_right;
    
    % 台車の位置を更新
    x = x + (v * cos(theta) - v_cross * sin(theta)) * dt;
    y = y + (v * sin(theta) + v_cross * cos(theta)) * dt;
    theta = theta + omega * dt;
    
    % theta を -pi から pi の範囲に正規化
    theta = mod(theta + pi, 2 * pi) - pi;
end

% 結果のプロット
figure;
subplot(3, 1, 1);
plot(x_history, y_history, 'b-', 'LineWidth', 2); hold on;
plot(x_front_history, y_front_history, 'c--', 'LineWidth', 1.5); % 前輪の軌跡
plot(x_history(1), y_history(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % 初期位置
plot(x_history(end), y_history(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % 終了位置
xlabel('X (m)');
ylabel('Y (m)');
title('台車の軌跡 (横断勾配あり)');
grid on;
axis equal;
legend('台車重心の軌跡', '前輪の軌跡', '初期位置', '終了位置');

% 左右後輪の速度プロット
subplot(3, 1, 2);
plot((0:dt:T-dt), v_left_history, 'b-', 'LineWidth', 1.5); hold on;
plot((0:dt:T-dt), v_right_history, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('後輪の速度 (m/s)');
title('左右後輪の速度');
legend('左後輪', '右後輪');
grid on;

% 台車の角度のプロット
subplot(3, 1, 3);
plot((0:dt:T-dt), theta_history, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('角度 \theta (rad)');
title('台車の角度の変化');
grid on;
