# Introdução de ROS e robôs móveis terrestres

Até o momento, fizemos o [primeiro subscritor](https://bitbucket.org/grupomecatronica/gmr_intro), mexemos com [parâmetros e roslaunch](https://bitbucket.org/grupomecatronica/gmr_intro_1) e vimos o [uso de POO em ROS](https://bitbucket.org/grupomecatronica/gmr_intro_2) quando fizemos a classe *RobotClass*. Agora utilizaremos as equações do robô diferencial para saber o estado do robô, seja em velocidade ou posição. 

## Publicador ROS

Faremos um publicador para disponibilizar a odometria, o estado do robô (posição e velocidade). Lembre-se da modularidade de ROS então queremos nós menores mas vários em paralelo. Assim, não faria sentido fazer o controle ou qualquer outra atividade que dependa da odometria dentro do mesmo nó. Opte, sempre que possível, pela interface subscritor-publicador. 

Utilizaremos o pacote *gmr_intro_poo* que começamos em [gmr_intro_2](https://bitbucket.org/grupomecatronica/gmr_intro_2). Vamos editar a nossa classe primeiro. Queremos publicar odometria portanto devemos criar um publicador de mensagens de tal tipo. Felizmente, já há uma mensagem padrão [nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html), cuja definição compacta é:

```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

Caso conheça o nome, mas não se lembre dos campos, basta usar o comando `rosmsg info nav_msgs/Odometry`

```console
user@pc:~/$ rosmsg info nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```

Trata-se de uma estrutura de dados, então se tivermos um `nav_msgs::Odometry odom`, para acessar a posição x devemos utilizar `odom.pose.pose.position.x`, por exemplo. Observe que `odom.pose.pose.orientation` é um [geometry_msgs/Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html), ou seja, quando quisermos definir a orientação do robô no espaço (a direção dele no plano, por exemplo), precisamos trabalhar com a forma de representação de quaternion. Um estudo à parte sobre quaternions e ângulos de Euler é interessante, porém para o objetivo de representar como o robô se encontra no espaço 2D só precisamos nos preocupar com a guinada/*yaw*, o ângulo de Euler relacionado à direção que o robô aponta. Tal ângulo foi descrito como &theta; em [gmr_intro_2](https://bitbucket.org/grupomecatronica/gmr_intro_2). Após os cálculos, [tf2](http://wiki.ros.org/tf2/Tutorials/Quaternions) fornece um método útil para a conversão de Euler para quaternion. 

Em gmr_intro_poo/include/gmr_intro_poo/gmr_intro_poo.hpp, vamos adicionar duas bibliotecas. Uma relacionada à mensagem que utilizaremos e outras com funções úteis para conversão de ângulos de Euler para quaternion:
```cpp
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
```

Faremos um loop explícito para o cálculo da odometria e sua publicação. O loop ficará em gmr_intro_poo_node.cpp, portanto adicionaremos um método público para realizar os cálculos de odometria e publicá-la:
```cpp
        void calculateOdom();
```

Adicionar um publicador como membro privado:
```cpp
        ros::Publisher _pub_odom;
```

Se lembrarmos das equações, temos `dt`, que é o tempo entre duas execuções consecutivas dos cálculos de odometria. Portanto, precisamos guardar o tempo (*timestamp*) da última execução e, assim, obter o valor de tempo decorrido `dt`. Portanto, mais um atributo privado:
```cpp
        ros::Time _prev_timestamp;
```
Além disso, as equações de cinemática acumulam a decomposição dos incrementos de velocidade por um tempo dt numa direção &theta; em x, y e &theta;. Assim, por conveniência, criaremos uma struct para armazenar esses valores:
```cpp
        struct RobotPose
        {
            double x;
            double y;
            double theta;
        }_robot_pose;
```

Vamos agora editar a implementação da classe, portanto o arquivo gmr_intro_poo/src/gmr_intro_poo.cpp. Primeiramente, precisamos inicializar o publicador e atribuir um valor inicial aos novos atributos. Essa segunda parte, embora seja redundante para a maioria dos usos (uso em desktop), é bastante importante verificar quando se trata de embarcar em módulos computacionais (uma Raspberry Pi, por exemplo). A compilação na Raspberry Pi (um ARM) é um processo diferente e não garante que as variáveis inicializem em zero. Isso é particularmente perigoso com variáveis que acumulam (veja equações da cinemática) ou que dependem de valores anteriores (cálculo de `dt`). No construtor da classe RobotClass::RobotClass(ros::NodeHandle* nh):

```cpp
    _pub_odom = _nh->advertise<nav_msgs::Odometry>("/odom", 1);
    _robot_pose = (const struct RobotPose) { 0 };
    _prev_timestamp = ros::Time::now();
```

 Informamos ao master que publicaremos uma mensagem do tipo <nav_msgs::Odometry> no tópico "/odom". E o segundo argumento indica o tamanho da fila de publicação. No nosso caso, queremos que o resto dos nós tenha acesso à informação mais atual possível, então colocamos 1. Note a inicialização de _robot_pose. É uma forma bastante compacta e funcional de inicializar uma struct com zeros. O valor de _prev_timestamp é iniciado com o tempo atual fornecido pelo tópico /clock de ROS. Em alguns casos é interessante utilizar `ros::WallTime::now()` porque como o próprio nome diz, utiliza o relógio de parede, portanto sincronizado com o tempo corrente. Já `ros::Time` utiliza /clock, que é facilmente manipulável. Um exemplo é a reprodução de rosbag, os arquivos de log de ROS. Na chamada de `rosbag play` e com o argumento `--clock`, é como se a máquina voltasse no tempo em que a rosbag for gravada. Com o argumento `-r 10`, a rosbag é reproduzida 10 vezes mais rápida. Além disso, é possível publicar no tópico /clock. Como o cálculo de `dt` é no mesmo tempo do conhecimento da velocidade dos motores, então deixaremos em `ros::Time` que serve tanto para uma execução em tempo corrente quanto manipulado. 
 
 Vamos agora criar o método com as equações que obtemos em [gmr_intro_2](https://bitbucket.org/grupomecatronica/gmr_intro_2):

```cpp
void RobotClass::calculateOdom()
{
    ros::Time cur_timestamp = ros::Time::now();
    double vl = _vel_m_s.left;
    double vr = _vel_m_s.right;
    double dt, wz, vx;
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;

    dt = cur_timestamp.toSec() - _prev_timestamp.toSec();
    _prev_timestamp = cur_timestamp;
    vx = 0.5*(vr + vl); // Eq. 1                     
    wz = (vr - vl)/_params.axle_track; // Eq.2
    _robot_pose.theta += wz*dt; // Eq. 8
    _robot_pose.x += vx*std::cos(_robot_pose.theta)*dt; // Eq. 6
    _robot_pose.y += vx*std::sin(_robot_pose.theta)*dt; // Eq. 7


    odom_quat.setRPY(0,0, _robot_pose.theta);
    odom.twist.twist.angular.z = wz;
    odom.twist.twist.linear.x = vx;
    odom.pose.pose.position.x = _robot_pose.x;
    odom.pose.pose.position.y = _robot_pose.y;
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);
    _pub_odom.publish(odom);
}
```

Temos alguns pontos importantes a notar. Se for usar valores que podem mudar enquanto a função é executada, armazene o valor numa variável como o caso de ros::Time::now().toSec(), _vel_m_s.left e _vel_m_s.right. É uma prática importante à medida que os cálculos ficam mais complexos e, portanto, mais demorados. Como &theta; é utilizada nas trê equações, é mais preciso atualizá-la primeiro e depois, x e y. É um modelo bastante simplificado, mas que atende aos propósitos de aprendizagem. Uma implementação bem completa de odometria pode ser encontrada no pacote [ackermann_steering_controlle](https://github.com/ros-controls/ros_controllers/blob/melodic-devel/ackermann_steering_controller/include/ackermann_steering_controller/odometry.h). Repare que tf2::toMsg(odom_quat) é necessária para converter de tf2::Quaternion para geometry_msgs::QUaternion.

RobotClass::calculateOdom() é um método que queremos chamar o mais rápido possível de forma a ter o menor `dt` possível, e assim, minimizar o erro acumulado devido ao modelo simplificado. Assim, chamaremos o método na mesma taxa em que os valores de velocidade de motores são atualizados. Para descobrir isso, podemos utilizar a linha de comando:

```console
user@pc: ~/$rostopic hz /left_rpm
```

com a qual descobrimos que a taxa de atualização é em torno de 50 Hz. Então, em gmr_intro_poo/src/gmr_intro_poo_node.cpp, definiremos o tempo de um loop, *loop_rate*:
```cpp
    ros::Rate loop_rate(50); //loop rate in Hz
```
Note que a linha acima deve ser colocada **depois** da criação do ros::NodeHandle. Caso contrário, uma exceção será gerada e o nó, finalizado.

E trocaremos ros::spin() por um loop explícito:
```cpp
    while(ros::ok())
    {
        robot.calculateOdom();
        loop_rate.sleep();
        ros::spinOnce();
    }
```

Enquanto ros::spin() gerava um loop interno e ficava constantemente se atualizando com o master, ros::spinOnce() é uma atualização pontual, só na hora em que é chamada. Por isso, é utilizada em loops explícitos, como o criado. `ros::ok()` verifica tanto se há algum sinal de ros::shutdown(), que pode ser manualmente configurado, quanto SIGINT, que é a interrupção por teclado (Ctrl + C). O código final fica:

Agora que editamos todos os arquivos, compile
```console
user@pc:~/ros_ws: $catkin_make
```
e rode o arquivo launch:
```console
user@pc:~/: $roslaunch gmr_intro_poo robot.launch
```
Em outra sessão de terminal, confira o resultado do tópico "/odom" com

```console
user@pc:~/: $rostopic echo /odom
```
Os códigos estão disponíveis em [gmr_intro_poo](https://bitbucket.org/grupomecatronica/gmr_intro_poo) no branch gmr_intro_3. Com isso, a nossa classe RobotClass adquiriu um publicador. Implementamos as equações de cinemática de robô com tração diferencial e temos a odometria disponível. No próximo tutorial, veremos sobre serviços.


Contato: Akihiro (akihirohh@gmail.com)

Apoio da Fundação de Amparo à Pesquisa do Estado de São Paulo (FAPESP) através do processo nº 2018/10894-2. 
As opiniões, hipóteses e conclusões ou recomendações expressas neste material são de responsabilidade do(s) autor(es) e não necessariamente refletem a visão da FAPESP.