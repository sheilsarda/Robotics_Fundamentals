x=zeros(1,5);
xind=1;
for t1=-1.4:0.01:1.4
    for t2=-1.2:0.01:1.4
        for t3=-1.8:0.01:1.7
            for t4=-1.5:0.01:1.7
                    x(1,xind)=cos(t1)*(197.325*cos(t3+t2)+146.05*sin(t2)+68*cos(t4+t3+t2));
                    y(1,xind)=sin(t1)*(187.325*cos(t3+t2)+146.05*sin(t2)+68*cos(t4+t3+t2));
                    z(1,xind)=146.05*cos(t2)+76.2-187*sin(t3+t2) +68*sin(t4+t3+t2);
                    xind=xind+1;
            end
        end
    end
end
