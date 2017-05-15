function Gp = createPlantModel(mass,fr,flag)
% flag == 1 刚体模型
% flag == 2 双质量块模型

switch flag
    case 1
        num = 1;
        den = [mass(1),0,0];
        Gp = tf(num,den);
    case 2 
        m1=mass(1);%台子质量
        m2=mass(2);%驱动器质量
        m=m1+m2;%双质量块模型共振频率
        f1=fr(1);
        w1=f1*2*pi;
        z1=0.00;
        G1=tf(1,[m,0,0]);
        G2=tf(w1*w1,[1,2*z1*w1,w1*w1]);
        Gp=G1*G2;
        
end

end
