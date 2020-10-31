clc;clear;

ind1=1;
%x-y plane
% for t2=-1.2:0.01:1.4
%     for t1=-1.4:0.01:1.4
%         x1(1,ind1)=cos(t1)*(197.325*cos((-pi/2)+t2)+146.05*sin(t2)+68*cos(0+(-pi/2)+t2));
%         y1(1,ind1)=sin(t1)*(187.325*cos((-pi/2)+t2)+146.05*sin(t2)+68*cos(0+(-pi/2)+t2));
%         ind1=ind1+1;
%     end
% end
% plot(x1,y1)
ind2=1;
%y-z plane


    for t2=-1.2:0.025:1.4
        for t3=-1.8:0.05:1.7
            for t4=-1.5:0.05:1.7
                for t1=-1.4:0.05:1.4
                    x(1,ind2)=cos(t1)*(197.325*cos(t3+t2)+146.05*sin(t2)+68*cos(t4+t3+t2));
                    y(1,ind2)=sin(t1)*(187.325*cos(t3+t2)+146.05*sin(t2)+68*cos(t4+t3+t2));
                    z(1,ind2)=146.05*cos(t2)+76.2-187*sin(t3+t2) +68*sin(t4+t3+t2);
                    ind2=ind2+1;
            end
        end
    end
end