function Gp = createPlantModel(mass,fr,flag)
% flag == 1 ����ģ��
% flag == 2 ˫������ģ��

switch flag
    case 1
        num = 1;
        den = [mass(1),0,0];
        Gp = tf(num,den);
    case 2 
        m1=mass(1);%̨������
        m2=mass(2);%����������
        m=m1+m2;%˫������ģ�͹���Ƶ��
        f1=fr(1);
        w1=f1*2*pi;
        z1=0.00;
        G1=tf(1,[m,0,0]);
        G2=tf(w1*w1,[1,2*z1*w1,w1*w1]);
        Gp=G1*G2;
        
end

end
