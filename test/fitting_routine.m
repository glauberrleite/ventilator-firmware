readings = readtable('list.csv', 'HeaderLines', 1)
dp = table2array(readings(:,3))
flow = table2array(readings(:,1))
%Certifique-se que dp não possui sinais de entrada iguais

i = 1
lengthofdp = length(dp)
while le(i, lengthofdp)
    j = i+1
    lengthofdp = length(dp)
    while le(j, lengthofdp)
        if dp(i)==dp(j)
            flow(i)=(flow(i)+flow(j))/2
            flow(j)=[]
            dp(j)=[]
        else
            j=j+1
        end
        lengthofdp = length(dp)
    end
    i=i+1
    lengthofdp = length(dp)
end

xx = -1:.1:1;
pp = spline(dp,flow,xx)
plot(xx, pp)