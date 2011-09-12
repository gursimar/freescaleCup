% This program gets data from serial and do the following processing on data
% 1. PLOT THE DATA
% 2. TAKE DIFF AND PLOT THE DATA
% 3. FIND CENTRE AND PLOT THE DATA

START = 4;

sen_val = zeros(1,128);
%figure;
h1 = subplot (2,1,1);
ylim ([0,350] );
%ylim ('auto');
xlim([1 128]);

% CREATE THE SERIAL IO
s = serial('COM41');
set(s,'BaudRate',115200);
set(s,'InputBufferSize',1024)
fopen(s);

% prepare the plot
%axes('xlim',[1,120],'ylim',[0,256]);
x=1:128;
%y=-inf*ones(size(x));
lh=line(x,sen_val,'marker','*','markersize',5,'linestyle','-');
%lh=line(x,sen_val,'marker','.','markersize',5,'linestyle','none');
%lb=line([inf,inf],[0,500]);
%cmap=jet(5);
%shg;

h2 = subplot (2,1,2);
ylim ([-100 100]);
%ylim ('auto');
xlim([1 128]);
lh2=line(x(1:end-1),sen_val(1:end-1),'marker','*','markersize',5,'linestyle','-');
hold on;
lh3=line(64,-15:0.1:15);
hold off;
%pause;

% infifnite loop to poll the sensor
try
    while 1
        %fetching the data
        fprintf(s,'6');
        out = fscanf(s);
        %size (out)
        %out (1)
        %out(241)
    
        % converting the data in decimal format
        for i = 1:128
            sen_val(i) = str2num(out((6*i)-5:(6*i)))
        end
        [max_sen_val,M_sen] = max (sen_val)
    
        % calculate the sum and difference
        %sensor_value_left = sum(sen_val(2:60));
        %sensor_value_right = sum(sen_val(61:120));
        %sensor_diff = sensor_value_left - sensor_value_right;
        
        
        
        dy_dx = diff(sen_val);  % apprantly its dy/dx 
        
        %[max_dydx,M] = max (dy_dx(15:115));
        %[min_dydx,m] = min (dy_dx(15:115));
        %centre = ((M+m)/2 ) + 15;
        
        [max_dydx,M] = max (dy_dx(START:end));        
        [min_dydx,m] = min (dy_dx(START:end));
                    
        centre = ((M+m)/2 );
        % SPECIFIED PROGRAM JOB ENDS HERE
        
        % ADDITIONAL JOBS HERE
        %max_dydx
        %min_dydx
    
        % plot  data
        set(lh,'xdata',x(START:end),'ydata',sen_val(START:end));
        %hold on;
        %ylim([0, max_sen_val+30]);
        %hold off;

        set(lh2,'xdata',x(START:end-1),'ydata',dy_dx(START:end));
        hold on;
        set(lh3,'xdata',centre,'ydata',0,'marker','*','markersize',20,'color','r');
        %ylim([min_dydx-40, max_dydx+40]);
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
