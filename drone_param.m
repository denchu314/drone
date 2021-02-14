% 2020.12.29 written by denchu
% dtanaka314@gmail.com
%
% drone control program
% 
close all;
clear;

%
% 機体パラメータ
% ニュートン・オイラー運動方程式により導出される機体状態
%



I_xx = 0.0075;     % x軸を中心とした慣性モーメント
I_yy = 0.0075;     % y軸を中心とした慣性モーメント
I_zz = 0.5;     % z軸を中心とした慣性モーメント

g = 9.80665;    %[m/s^2] 重力加速度 
J_r = 0.0000005;       % ロータ・プロペラの慣性モーメント


pitch_init = 10.0/360 * 2 * pi;   %[rad] ピッチ初期値
roll_init = 10.0/360 * 2 * pi;    %[rad] ロール初期値
yaw_init = 0.0/360 * 2 * pi;     %[rad/s] ヨー初期値

x_init = 0.0;       %[m] 機体重心位置初期値
y_init = 0.0;       %[m] 機体重心位置初期値
z_init = 0.0;       %[m] 機体重心位置初期値

N1_rad_init = 0.0;  %[rad/s] プロペラ1初期回転速度
N2_rad_init = 0.0;  %[rad/s] プロペラ2初期回転速度
N3_rad_init = 0.0;  %[rad/s] プロペラ3初期回転速度
N4_rad_init = 0.0;  %[rad/s] プロペラ4初期回転速度


% D0605データ:12000rpm/V, 推力48g, 消費電力28.9W, 消費電流3.9A,
% Lipo2S:7.4V, プロペラ Φ=56mm より

l = 0.05;       %[m] 機体重心からロータ回転軸までの長さ
m = 0.15;       %[kg] 機体重量
A = 0.048 * g/(2 * pi * 7.4 * 12000 / 60)^2;  %[N/(rad/s)^2] 推力/プロペラ回転速度^2
d = 0.000000001;       % 反トルク係数

%https://akizukidenshi.com/download/ds/bosch/BST-BMX055-DS000.pdfより
d_deg_gyrosensor_resolution = 0.0038 / 360 * 2 * pi; %[rad/s] ジャイロセンサの分解能
f_gyrosensor_samplerate = 2000; %[Hz] 加速度センサのサンプルレート

deg_magsensor_resolution = 1 / 360 * 2 * pi; %[rad] 地磁気センサの分解能?
f_magsensor_samplerate = 20; %[Hz] 地磁気センサのサンプルレート

acc_accsensor_resolution = 0.00098 * g; %[m/s^2] 加速度センサの分解能
f_accsensor_samplerate = 1000; %[Hz] 加速度センサのサンプルレート

K_d_angle = 0; %[-] 係数 ジャイロセンサvs地磁気センサの比率

%PID制御係数
K_p_roll = 10.0; %[-] ロール比例係数
K_i_roll = 2.0; %[-] ロール積分係数 
K_d_roll = 2.0; %[-] ロール微分係数

K_p_pitch = 10.0; %[-] ピッチ比例係数
K_i_pitch = 2.0; %[-] ピッチ積分係数 
K_d_pitch = 2.0; %[-] ピッチ微分係数 

K_p_yaw = 0.0; %[-] ヨー比例係数
K_i_yaw = 0.00; %[-] ヨー積分係数 
K_d_yaw = 0.0; %[-] ヨー微分係数

K_p_throttle = -5.0; %[-] スロットル比例係数

T_0 = 78490 / 60 * 2 * pi; %[rad/s] 基底回転速度（ホバリング時回転速度）
%T_0 = 79000 / 60 * 2 * pi; %[rad/s] 基底回転速度（ホバリング時回転速度）
dT_max = 19200 / 60 * 2 * pi; % [rad/s] 回転速度増加値