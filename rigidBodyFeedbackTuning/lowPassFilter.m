function lowPass=lowPassFilter(wn,z)
[num,den]=ord2(wn,z);
temp=tf(num,den);
temp=temp*wn*wn;
lowPass=temp;
end