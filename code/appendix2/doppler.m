% RETIOT Velocity Detection: Simulation of velocity  detection

%% Initialization
clear ; close all; clc
%%
% 
% 

fc=77*10^9;          % 77 GHz
BW=4*10^9;           % 4 GHz
Tc=40*10^-6;         % 40 us
S=BW/Tc;             % 
c=3*10^8;           %m/s
N=20000000;        % # samples
dt=Tc/N;            % Ts
t=[0:dt:Tc];        % time vector

my_vector=[];

N_chirps=64;
velocityMax=c*pi/(fc*4*pi*Tc);
distance_start=5.0;
velocity_object=7.0; %m s

dist_obj(1)=distance_start;
for i=2:N_chirps
    dist_obj(i)=dist_obj(i-1)+velocity_object*Tc;
end

%% N Chirps 

f1=fc+S*t;              %TX signal frequency     
TX_signal=sin(2.*pi.*f1.*t);

for i=1:N_chirps
    round_trip_delay=2*dist_obj(i)/c;  % s
    n_zeros=round(round_trip_delay/dt);    % RX signal
    tmp=length(t)-n_zeros;
    f2=[zeros(1,n_zeros) f1(1:tmp)];
    offset=2*pi*fc*dt*n_zeros;
    RX_signal=0.1*sin(2.*pi.*f2.*t-offset);
    RX_signal(1:n_zeros)=0;
    IF_signal=RX_signal.*TX_signal;
    
    %% IF_signal sampling
    B = fir1(70,0.1,'low');
    IF_signal_F=filter(B,1,IF_signal);
    Fs_ADC=4306*10^4;
    Ts_ADC=1/Fs_ADC;

    n_skip=round(Ts_ADC/dt); %skip some samples
    tmp=1:n_skip:length(t);
    IF_signal_sampled=IF_signal_F(tmp);
    
    %% FFT IF signal
    N=256;
    F=fft(IF_signal_sampled,N);
    P2 = abs(F);
    P1 = P2(1:N/2+1);
    P2_ang = angle(F);
    P1_ang = P2_ang(1:N/2+1);

    Threshold=3.5;
    idx=find(P1>Threshold);

    phasor(i)= F(idx);
end

%% FFT IF signal
doppler_FFT=fft(phasor);

dw=2*pi/length(phasor);
w=[0:dw:2*pi-dw];

stem(w,abs(doppler_FFT));
xlabel('\omega (rad/s)');
ylabel('Amplitude FFT');
title('Doppler FFT phase differances');

vres=c*dw/(fc*4*pi*Tc)
vmax=c*pi/(fc*4*pi*Tc)
idx=find(abs(doppler_FFT)==max(abs(doppler_FFT)));
v_calc=c*w(idx)/(fc*4*pi*Tc)
v=[0:vres:2*vmax-vres];


figure;
stem(v,abs(doppler_FFT));
xlabel('velocity (m/s)');
ylabel('Amplitude FFT');
title('Doppler FFT velocity estimation');

