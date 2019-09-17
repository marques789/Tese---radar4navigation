% RETIOT Range Detection: Simulatio of object range detection

%% Initialization
clear ; close all; clc

fc=77*10^9;          % 77 GHz
BW=4*10^9;           % 4 GHz
Tc=40*10^-6;         % 40 us
S=BW/Tc;             % 

dist_obj=[0.0469+0.0469/2 1.1 3.0 5 5.3];         %m
n_objects=length(dist_obj);
c=3*10^8;           %m/s
round_trip_delay=2*dist_obj/c;  % s

N=10000000;        % # samples
dt=Tc/N;            % Ts
t=[0:dt:Tc];        % time vector

%%
f1=fc+S*t;
TX_signal=sin(2.*pi.*f1.*t);

n_zeros=round(round_trip_delay/dt);    % # samples
%%
plot(t(1:10:10000),f1(1:10:10000));
RX_signal=zeros(1,length(TX_signal));
for i=1:n_objects
tmp=length(TX_signal)-n_zeros(i);
f2=[zeros(1,n_zeros(i)) f1(1:tmp)];
RX_signal_i=0.1*sin(2.*pi.*f2.*t);
RX_signal=RX_signal+RX_signal_i;
plot(t(1:10:10000),f2(1:10:10000));
hold on;
end

axis([0 4*10^-8 fc fc+4*10^6]);
%%

IF_signal=RX_signal.*TX_signal;

%%
B = fir1(70,0.1,'low');
IF_signal_F=filter(B,1,IF_signal);
Fs_ADC=8*10^6;
Ts_ADC=1/Fs_ADC;
n_skip=round(Ts_ADC/dt);
tmp=1:n_skip:length(t);
IF_signal_sampled=IF_signal_F(tmp);
%plot(IF_signal_sampled(1:1000));

L = length(IF_signal_sampled);             % Length of signal
N=256;

F=fft(IF_signal_sampled,N);
P2 = abs(F);
P1 = P2(1:N/2+1);
%P1(2:end-1) = 2*P1(2:end-1);

f3 = Fs_ADC*(0:(N/2))/N;
figure;
stem(f3/10^6,P1); 
title('Single-Sided Amplitude Spectrum of IF')
xlabel('f (MHz)')
ylabel('|P1(f)|')
f3(2)
dres=f3(2)*c/(2*S)
fmax=Fs_ADC/2
dmax=fmax*c/(2*S)

Threshold=3.5;
idx=find(P1>Threshold);
number_objects_found=length(idx);
for i=1:number_objects_found
    dcalc_obj(i)=f3(idx(i))*c/(2*S);
end
%%
dres=f3(2)*c/(2*S);
d=[0:dres:dres*length(f3)-dres];
figure;
stem(d,P1); 


