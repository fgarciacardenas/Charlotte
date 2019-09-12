void paso(double& modulo, double& residuo, double& num, double& distancia) {
    if (distancia > =  4) {
        
        modulo  =  4;
        double r  =  distancia % modulo;

        if (r > =  0.5) {
            residuo  =  r;
        }
        else {
            residuo  =  0;
        }
        
        num  =  floor((distancia - residuo)/modulo);
    }
    if (distancia > =  0.5 && distancia < 4) {
        
        modulo  =  0.5;
        double r  =  distancia % modulo;
        
        if (r > =  0.5) {
            residuo  =  r;
        }
        else {
            residuo  =  0;
        }
    
        num  =  floor((distancia-residuo)/modulo);
    }
    if (distancia < 0.5) {
        modulo  =  0;
        residuo  =  0;
        num  =  0;
    }
}


void vectores(double& angulo, double& dx, double& dy, double& distancia, Vector2d& A, Vector2d& B) {
    angulo  =  atand((B(2) - A(2))/(B(1) - A(1)));
    Dx  =  B(1) - A(1);
    
    if (Dx < 0) {
        dx  =  -1;
    }
    else {
        dx  =  1;
    }

    Dy = B(2)-A(2);
    
    if (Dy<0) {
        dy  =  -1;
    }
    else {
        dy  =  1;
    }

    distancia  =  sqrt(Dx^2 + Dy^2);
}


void select(double& casos, double& director) {
    if (director(1)  =  =  1 && director(2)  =  =  1) {
        casos  =  1;
    }
    else if (director(1)  =  =  -1 && director(2)  =  =  1) {
        casos  =  2;
    }
    else if (director(1)  =  =  -1 && director(2)  =  =  -1) {
        casos  =  3;
    }
    else if (director(1)  =  =  1 && director(2)  =  =  -1) {
        casos  =  4; 
    }
}


int main() {
    // Se inicializan las variables
    double casos;
    double angulo, dx, dy, distancia;
    double modulo, residuo, num, distancia;
    Vector2d A, B, director;
    VectorXd M;

    // Para el paso
    A << (1, 1); 
    B << (12, -10);
    vectores(double& angulo, double& dx, double& dy, double& distancia, Vector2d& A, Vector2d& B);
    
    director << (dx, dy);
    paso(double& modulo, double& residuo, double& num, double& distancia);
    
    parax  =  modulo*cosd(angulo)*director(1);
    paray  =  modulo*sind(angulo)*director(2);
    M << (30, 30, -30, 30, -30, -30, 30, -30, 0, 0);

    casos  =  select(director);

    switch(casos) {
        case 1:
            M(1) = M(1)+parax/2;M(2) = M(2)+paray/2;
            M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
            M(5) = M(5)+parax/2;M(6) = M(6)+paray;
            for (n = 0:num-1) {
                M(3) = M(3)+parax;M(4) = M(4)+paray;
                x1(n+1) = M(3);y1(n+1) = M(4);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx2(n+1) = M(9);cy2(n+1) = M(10);
                M(7) = M(7)+parax;M(8) = M(8)+paray;
                x4(n+1) = M(7);y4(n+1) = M(8);
                 
                M(1) = M(1)+parax;M(2) = M(2)+paray;
                x2(n+1) = M(1);y2(n+1) = M(2);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx(n+1) = M(9);cy(n+1) = M(10);
                M(5) = M(5)+parax;M(6) = M(6)+paray;
                x3(n+1) = M(5);y3(n+1) = M(6);
            }
             
        case 2:
            M(3) = M(3)+parax/2;M(4) = M(4)+paray/2;
            M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
            M(7) = M(7)+parax/2;M(8) = M(8)+paray;
            for (n = 0:num-1) {
                M(1) = M(1)+parax;M(2) = M(2)+paray;
                x1(n+1) = M(1);y1(n+1) = M(2);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx2(n+1) = M(9);cy2(n+1) = M(10);
                M(5) = M(5)+parax;M(6) = M(6)+paray;
                x4(n+1) = M(5);y4(n+1) = M(6);
                
                M(3) = M(3)+parax;M(4) = M(4)+paray;
                x2(n+1) = M(3);y2(n+1) = M(4);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx(n+1) = M(9);cy(n+1) = M(10);
                M(7) = M(7)+parax;M(8) = M(8)+paray;
                x3(n+1) = M(7);y3(n+1) = M(8);
            }
        
        case 3:
            M(5) = M(5)+parax/2;M(6) = M(6)+paray/2;
            M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
            M(1) = M(1)+parax/2;M(2) = M(2)+paray;
            for (n = 0:num-1) { 
                M(7) = M(7)+parax;M(8) = M(8)+paray;
                x1(n+1) = M(7);y1(n+1) = M(8);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx2(n+1) = M(9);cy2(n+1) = M(10);
                M(3) = M(3)+parax;M(4) = M(4)+paray;
                x4(n+1) = M(3);y4(n+1) = M(4);
                
                M(5) = M(5)+parax;M(6) = M(6)+paray;
                x2(n+1) = M(5);y2(n+1) = M(6);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx(n+1) = M(9);cy(n+1) = M(10);
                M(1) = M(1)+parax;M(2) = M(2)+paray;
                x3(n+1) = M(1);y3(n+1) = M(2);
            }
             
        case 4:
            M(7)  =  M(7)+parax/2;M(8) = M(8)+paray/2;
            M(9)  =  M(9)+parax/4;M(10) = M(10)+paray/4;
            M(3)  =  M(3)+parax/2;M(4) = M(4)+paray;
            for (n = 0:num-1) {
                M(5) = M(5)+parax;M(6) = M(6)+paray;
                x1(n+1) = M(5);y1(n+1) = M(6);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx2(n+1) = M(9);cy2(n+1) = M(10);
                M(1) = M(1)+parax;M(2) = M(2)+paray;
                x4(n+1) = M(1);y4(n+1) = M(2);
                 
                M(7) = M(7)+parax;M(8) = M(8)+paray;
                x2(n+1) = M(7);y2(n+1) = M(8);
                M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
                cx(n+1) = M(9);cy(n+1) = M(10);
                M(3) = M(3)+parax;M(4) = M(4)+paray;
                x3(n+1) = M(3);y3(n+1) = M(4);
             }
    }

}









clear;clc;close;

%PARA EL PASO

A = [1 1]; B = [-20 -10];%%Punto al que se debe desplazar

%PASOS PREVIOS QUE SERÁN MODIFICADOS
[angulo,dx,dy,distanciatotal] = vectores(A,B);%Halla los valores de angulo
%directores y distancia total a recorrer
director = [dx dy]; %Poner director en una matriz
[modulo1,residuo1,num1] = paso(distanciatotal);
%y el numero de pasos
parax1 = modulo1*cosd(angulo)*director(1);%Desplazamiento en x en funcion
%del angulo
paray1 = modulo1*sind(angulo)*director(2);%Desplazamiento en y en funcion
%del angulo
%VALORES QUE SE USAN A LO LARGO DEL CÓDIGO
distancia = recalcular(distanciatotal,parax1,paray1);%Recalcular
[modulo,residuo,num] = paso(distancia);%Da el modulo de paso, el residuo
parax = modulo*cosd(angulo)*director(1);%Desplazamiento en x en funcion
%del angulo
paray = modulo*sind(angulo)*director(2);%Desplazamiento en y en funcion
%del angulo
resix = residuo*cosd(angulo)*director(1);
resiy = residuo*sind(angulo)*director(2);


%SE CENTRARÁ A CHARLOTTE
%Datos Chtt
L = [30 30 -30 30 -30 -30 30 -30 A(1) A(2)];%Posiciones leídas
%Datos Chtt
Base_L = 15.5;
r_1 = 5.5;
r_2 = 7.5;
r_3 = 22.5;

%Centrar
Dcog = (Base_L*sqrt(2))/2;
Drod = 5.5+(7.5/sqrt(2));
Dres = Dcog+Drod;

L(1) = L(9)+Dres*sqrt(2);L(2) = L(10)+Dres*sqrt(2);
L(3) = -1*(L(9)+(Dres*sqrt(2)));L(4) = L(10)+Dres*sqrt(2);
L(5) = -1*(L(9)+(Dres*sqrt(2)));L(6) = -1*(L(10)+Dres*sqrt(2));
L(7) = L(9)+(Dres*sqrt(2));L(8) = -1*(L(10)+Dres*sqrt(2));

%POSICIONES CENTRADAS
M = L;%Posiciones centradas

%CAMINATA POR CASOS

casos = Select(director);

switch casos
    case 1 %PRIMER CUADRANTE
         M(1) = M(1)+parax/2;M(2) = M(2)+paray/2;
         M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
         M(5) = M(5)+parax/2;M(6) = M(6)+paray;
         for n = 0:num-1
             M(3) = M(3)+parax;M(4) = M(4)+paray;
             x2(n+1) = M(3);y2(n+1) = M(4);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx2(n+1) = M(9);cy2(n+1) = M(10);
             M(7) = M(7)+parax;M(8) = M(8)+paray;
             x4(n+1) = M(7);y4(n+1) = M(8);
             
             M(1) = M(1)+parax;M(2) = M(2)+paray;
             x1(n+1) = M(1);y1(n+1) = M(2);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx(n+1) = M(9);cy(n+1) = M(10);
             M(5) = M(5)+parax;M(6) = M(6)+paray;
             x3(n+1) = M(5);y3(n+1) = M(6);
         end
         
         M(3) = M(3)+resix;M(4) = M(4)+resiy;
         x2(num+1) = M(3);y2(num+1) = M(4);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx2(num+1) = M(9);cy2(num+1) = M(10);
         M(7) = M(7)+resix;M(8) = M(8)+resiy;
         x4(num+1) = M(7);y4(num+1) = M(8);
             
         M(1) = M(1)+resix;M(2) = M(2)+resiy;
         x1(num+1) = M(1);y1(num+1) = M(2);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx(num+1) = M(9);cy(num+1) = M(10);
         M(5) = M(5)+resix;M(6) = M(6)+resiy;
         x3(num+1) = M(5);y3(num+1) = M(6);
         
         plot(x1,y1,'o',x2,y2,'o',x3,y3,'o',x4,y4,'o',cx2,cy2,'*');
         legend('Pata 1','Pata 2','Pata 3', 'Pata 4', 'Centro de gravedad final');
         disp('Primer cuadrante');
         
    case 2 %CUADRANTE 2
         M(3) = M(3)+parax/2;M(4) = M(4)+paray/2;
         M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
         M(7) = M(7)+parax/2;M(8) = M(8)+paray;
         for n = 0:num-1
             M(1) = M(1)+parax;M(2) = M(2)+paray;
             x1(n+1) = M(1);y1(n+1) = M(2);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx2(n+1) = M(9);cy2(n+1) = M(10);
             M(5) = M(5)+parax;M(6) = M(6)+paray;
             x3(n+1) = M(5);y3(n+1) = M(6);
             
             M(3) = M(3)+parax;M(4) = M(4)+paray;
             x2(n+1) = M(3);y2(n+1) = M(4);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx(n+1) = M(9);cy(n+1) = M(10);
             M(7) = M(7)+parax;M(8) = M(8)+paray;
             x4(n+1) = M(7);y4(n+1) = M(8);
         end
         
         M(1) = M(1)+resix;M(2) = M(2)+resiy;
         x1(num+1) = M(1);y1(num+1) = M(2);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx2(num+1) = M(9);cy2(num+1) = M(10);
         M(5) = M(5)+resix;M(6) = M(6)+resiy;
         x3(num+1) = M(5);y3(num+1) = M(6);
             
         M(3) = M(3)+resix;M(4) = M(4)+resiy;
         x2(num+1) = M(3);y2(num+1) = M(4);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx(num+1) = M(9);cy(num+1) = M(10);
         M(7) = M(7)+resix;M(8) = M(8)+resiy;
         x4(num+1) = M(7);y4(num+1) = M(8);
         
         plot(x1,y1,'o',x2,y2,'o',x3,y3,'o',x4,y4,'o',cx2,cy2,'*');
         legend('Pata 1','Pata 2','Pata 3', 'Pata 4', 'Centro de gravedad final');
         disp('Segundo cuadrante');
    
    case 3 
         M(5) = M(5)+parax/2;M(6) = M(6)+paray/2;
         M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
         M(1) = M(1)+parax/2;M(2) = M(2)+paray;
         for n = 0:num-1
             M(7) = M(7)+parax;M(8) = M(8)+paray;
             x4(n+1) = M(7);y4(n+1) = M(8);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx2(n+1) = M(9);cy2(n+1) = M(10);
             M(3) = M(3)+parax;M(4) = M(4)+paray;
             x2(n+1) = M(3);y2(n+1) = M(4);
             
             M(5) = M(5)+parax;M(6) = M(6)+paray;
             x3(n+1) = M(5);y3(n+1) = M(6);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx(n+1) = M(9);cy(n+1) = M(10);
             M(1) = M(1)+parax;M(2) = M(2)+paray;
             x1(n+1) = M(1);y1(n+1) = M(2);
         end
        M(7) = M(7)+resix;M(8) = M(8)+resiy;
         x4(num+1) = M(7);y4(num+1) = M(8);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx2(num+1) = M(9);cy2(num+1) = M(10);
         M(3) = M(3)+resix;M(4) = M(4)+resiy;
         x2(num+1) = M(3);y2(num+1) = M(4);
             
         M(5) = M(5)+resix;M(6) = M(6)+resiy;
         x3(num+1) = M(5);y3(num+1) = M(6);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx(num+1) = M(9);cy(num+1) = M(10);
         M(1) = M(1)+resix;M(2) = M(2)+resiy;
         x1(num+1) = M(1);y1(num+1) = M(2);
         
         plot(x1,y1,'o',x2,y2,'o',x3,y3,'o',x4,y4,'o',cx2,cy2,'*');
         legend('Pata 1','Pata 2','Pata 3', 'Pata 4', 'Centro de gravedad final');
         disp('Tercer cuadrante');
         
case 4
         M(7) = M(7)+parax/2;M(8) = M(8)+paray/2;
         M(9) = M(9)+parax/4;M(10) = M(10)+paray/4;
         M(3) = M(3)+parax/2;M(4) = M(4)+paray;
         for n = 0:num-1
             M(5) = M(5)+parax;M(6) = M(6)+paray;
             x3(n+1) = M(5);y3(n+1) = M(6);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx2(n+1) = M(9);cy2(n+1) = M(10);
             M(1) = M(1)+parax;M(2) = M(2)+paray;
             x1(n+1) = M(1);y1(n+1) = M(2);
             
             M(7) = M(7)+parax;M(8) = M(8)+paray;
             x4(n+1) = M(7);y4(n+1) = M(8);
             M(9) = M(9)+parax/2;M(10) = M(10)+paray/2;
             cx(n+1) = M(9);cy(n+1) = M(10);
             M(3) = M(3)+parax;M(4) = M(4)+paray;
             x2(n+1) = M(3);y2(n+1) = M(4);
         end
         M(5) = M(5)+resix;M(6) = M(6)+resiy;
         x3(num+1) = M(5);y3(num+1) = M(6);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx2(num+1) = M(9);cy2(num+1) = M(10);
         M(1) = M(1)+resix;M(2) = M(2)+resiy;
         x1(num+1) = M(1);y1(num+1) = M(2);
             
         M(7) = M(7)+resix;M(8) = M(8)+resiy;
         x4(num+1) = M(7);y4(num+1) = M(8);
         M(9) = M(9)+resix/2;M(10) = M(10)+resiy/2;
         cx(num+1) = M(9);cy(num+1) = M(10);
         M(3) = M(3)+resix;M(4) = M(4)+resiy;
         x2(num+1) = M(3);y2(num+1) = M(4);
         
         plot(x1,y1,'o',x2,y2,'o',x3,y3,'o',x4,y4,'o',cx2,cy2,'*');
         legend('Pata 1','Pata 2','Pata 3', 'Pata 4', 'Centro de gravedad final');
         disp('Cuarto cuadrante'); 
end
    



