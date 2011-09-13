% This script 

%Some Constants
START = 4;
LB_Width = 30;
UB_Width = 50;
MIN_FINGER = 25;
FINGER = 7;
RIGHT_GUARD = 85;
LEFT_GUARD = 45;
DIFF =0;

% Other variables
s = serial('COM41');
set(s,'BaudRate',115200);
set(s,'InputBufferSize',1024)
fopen(s);

sen_val = zeros(1,128);
h1 = subplot (2,1,1);
%ylim ([0,256] );
ylim ('auto');
xlim([1 128]);

% Prepare the plot
x=1:128;
lh=line(x,sen_val,'marker','*','markersize',5,'linestyle','-');

h2 = subplot (2,1,2);
%ylim ([-50 50]);
ylim ('auto');
xlim([1 128]);
lh2=line(x(1:end-1),sen_val(1:end-1),'marker','*','markersize',5,'linestyle','-');
hold on;
lh3=line(64,-15:0.1:15);
hold off;
%pause;

% Infifnite loop to poll the sensor
try
    while 1
% Fetching the data
        fprintf(s,'6');
        out = fscanf(s);
    
% Converting the data in decimal format
        for i = 1:128
            sen_val(i) = str2num(out((6*i)-5:(6*i)));
        end
        centre = hex2dec(out(769:770));

% Taking differentials        
        dy_dx = diff(sen_val);  % apprantly its dy/dx 

% Plot  results
        set(lh,'xdata',x(START:end),'ydata',sen_val(START:end));
        set(lh2,'xdata',x(START:end-1),'ydata',dy_dx(START:end));
        hold on;
        set(lh3,'xdata',centre,'ydata',0,'marker','*','markersize',20,'color','r');
        hold off;
        
        pause(.0001); % <- a time consuming OP
        %pause
    end
catch
    disp('--> coninuous loop was manually interrupted')
end
    
% clear the data
fclose(s)
delete(s)
clear s
