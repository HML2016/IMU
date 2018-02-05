%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% brief:CQYD_CS
% File Name: main.m
% Project Name:PDR_BISTU
% Author:Liuning
% Date:20150105
% CopyRight:LiuNing
% Description:
%% 系统初始化，清除系统内存
clear all;
clc;
close all;
%% 导入惯导数据
disp('1.导入算法惯性测量单元数据')
u=settings();  %u为输出惯导数据矩阵，
%% 运行零速检测 
disp('2.运行零速检测')
[zupt T]=zero_velocity_detector(u);
%% 运行kalman滤波
disp('3.运行卡尔曼滤波')
[x_h cov]=ZUPTaidedINS(u,zupt);
%% 结果显示 
disp('4.显示结果')
view_data;

