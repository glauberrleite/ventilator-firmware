readings = readtable('list.csv', 'HeaderLines', 1)
dp = table2array(readings(:,3))
flow = table2array(readings(:,1))
%Certifique-se que dp não possui sinais de entrada iguais

for i = 1:1:length(dp)
    for j = i+1:1:length(dp)
        if dp(i)==dp(j)
            flow(i)=(flow(i)+flow(j))/2
            flow(j)=[]
            dp(j)=[]
            j=j-1
        end
    end
end

pp = spline(dp,flow)
xx = -1:.1:1;
plot(dp, flow)