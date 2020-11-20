# Introdução de ROS e robôs móveis terrestres

*Observação: não é necessário clonar este repositório.*

Criamos o [subscritor](https://bitbucket.org/grupomecatronica/gmr_intro) para o tópico */left_rpm*. Porém, temos o */right_rpm* também. O que precisamos para conseguir a informação desse último sem precisar escrever outro nó? Neste tutorial, veremos um pouco sobre parâmetros e roslaunch.

*Preparação:* deixe rodando `roscore` e `rosrun gmr_intro gmr_intro_node`.

## Utilizando parâmetros [C++]

Os parâmetros ROS possibilitam a troca de valor de variáveis dos nós sem a necessidade de recompilação. Eles podem ser acessados do servidor de parâmetros, que é executado junto de rosmaster. Através da linha de comando `$ rosparam list`, podemos ver alguns parâmetros disponíveis, por exemplo, */axle_track*, */gear_ratio* e */wheel_radius*. Note que os parâmetros continuarão disponíveis enquanto o master estiver ativo (i.e. um `roscore` rodando). Podemos obter o seu valor com `rosparam get <param>`, por exemplo `rosparam get /axle_track`. Mas o que nos interessa agora é acessá-los dentro do nó. Para isso, podemos utilizar dois métodos: ros::NodeHandle::getParam() ou ros::param::get().

Atualizando nosso subscritor em ~ros_ws/src/sub_rpm/src/sub_rpm_node.cpp:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

void subCb(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO_STREAM(msg->data);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh;

    std::string nome_topico = "/left_rpm";
    int tamanho_fila = 10;
    nh.getParam("nome_topico", nome_topico);
    nh.getParam("tamanho_fila", tamanho_fila);
    //nh.param<std::string>("nome_topico", nome_topico, "/left_rpm");
    //nh.param("tamanho_fila", tamanho_fila, 10);
    ROS_INFO_STREAM("nome_topico: " << nome_topico << "tamanho_fila: " << tamanho_fila);
    ros::Subscriber sub_rpm = nh.subscribe \
        (nome_topico, tamanho_fila, subCb);

    ros::spin();
}
```

observe que 
```cpp
    nh.getParam("tamanho_fila", tamanho_fila);
    nh.getParam("nome_topico", nome_topico);
```
procuram um parâmetro com o nome correspondente ao primeiro argumento e armazena o valor encontrado na variável passada (por endereço) no segundo argumento. Caso o parâmetro não exista no servidor, as variáveis manterão o valor anterior, no caso, */left_rpm* e *10*. Note que a outra opção é com ros::param():

```cpp    
    nh.param<std::string>("nome_topico", nome_topico, "/left_rpm");
    nh.param("tamanho_fila", tamanho_fila, 10);
```

em que podemos definir um valor padrão caso o parâmetro não seja encontrado no servidor. Embora neste pequeno exemplo as duas opções levem ao mesmo resultado (valor padrão de */left_rpm* e *10*), o uso de ros::param() é preferível devido ao maior controle e facilidade de leitura do código, principalmente quando há um número maior de parâmetros.

Agora, se queremos nos subscrever a */right_rpm*, podemos `$ rosparam set nome_topico /right_rpm` e de forma análoga caso queiramos voltar a */left_rpm*.

**NOTA IMPORTANTE:** o sistema de nomes em ROS pode ser um pouco confuso. Em `nh.getParam("nome_topico", nome_topico);`, procuramos por "nome_topico", mas quando vamos atribuir um valor a ele com `rosparam set`, utilizamos "/nome_topico". Por que isso acontece? O motivo é que inicializamos `nh` como um ros::NodeHandle "público"  e também devido à presença/ausência de "/" nos nomes. Um nome que inicia com "/" indica que é o nome final, independente de como o nó for executado. Se o nome não possui "/" no começo, é um nome que pode ter prefixos. Um exemplo será dado até o fim deste tutorial. Voltando ao primeiro motivo, como é um ros::NodeHandle público, todos os nomes usados por `nh` são tratados como finais, portanto há um "/" implícito no começo de todos os nomes. Para termos maior controle, iniciaremos `nh` de forma "privada", substituindo `ros::NodeHandle nh;` por `ros::NodeHandle nh("~");`. Desta forma, os nomes (exceto aqueles que já começam com "/") serão resolvidos de acordo com a posição do nó dentro da hierarquia de nomes de ROS.

## Uso de roslaunch

O roslaunch é uma ferramenta que permite facilmente rodar vários nós e também configurar os parâmetros. Caso não existisse, teríamos que abrir um terminal para cada nó e configurar cada parâmetro com `rosparam set`. Vamos criar um arquivo launch para imprimir os valores de */left_rpm* e /right_rpm*.

Primeiro, vamos fazer algumas alterações no nosso subscritor: mudança do ros::NodeHandle de "público" para "privado"; e um incremento em ROS_INFO_STREAM para sabermos qual nó está imprimindo.

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

void subCb(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO_STREAM(ros::this_node::getName() << " ouvindo " << msg->data);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh("~");

    std::string nome_topico = "/left_rpm";
    int tamanho_fila = 10;    
    nh.param("tamanho_fila", tamanho_fila, 10);
    nh.param<std::string>("nome_topico", nome_topico, "/left_rpm");
  
    ros::Subscriber sub_rpm = nh.subscribe \
        (nome_topico, tamanho_fila, subCb);

    ros::spin();
}
```

Vamos então criar uma pasta chamada launch dentro de sub_rpm e dentro dela, um arquivo XML chamado primeiro.launch cujo conteúdo é mostrado abaixo:

```xml
<launch>

  <node pkg="gmr_intro" type="gmr_intro_node" name="gmr_pub_vel">
  </node>
  
  <node pkg="sub_rpm" type="sub_rpm_node" name="sub_left" output="screen">
    <param name="nome_topico" value="/left_rpm"/>
  </node>

  <node pkg="sub_rpm" type="sub_rpm_node" name="sub_right" output="screen">
    <param name="nome_topico" value="/right_rpm"/>
  </node>

</launch>
```

Note que estamos iniciando três nós: o gmr_intro_node e mais dois sub_rpm_node. Em cada caso, devemos informar o pacote em `pkg`, o nome do executável (aquele que colocamos no CMakeLists.txt) em `type` e, finalmente, um nome único para o nó em `name`. Esse último nome substitui o nome que colocamos em *ros::init(argc, argv, "nome_no")* no começo da função main(). Observe que na chamada dos nós *sub_left* e *sub_right* temos a definição de um parâmetro, o *nome_topico*, cujo valor é atribuído casa o nó chamado com o tópico a ser lido. E também temos `output="screen"`. Isso indica que a saída do nó (e.g. o ROS_INFO_STREAM) aparecerá na tela. Caso não queira, é só apagar essa parte no launch.

Para executar o launch file, basta 

```console
user@pc: ~/$ roslaunch sub_rpm primeiro.launch
```

não sendo necessário rodar `roscore` antes porque o próprio launch abre um master ao ser executado caso não exista. Com os `output="screen"` ligados, esperamos uma sequência de ROS_INFO_STREAM indicando alternadamente (ou próximo disso) *sub_left* e *sub_right* ouvindo algo.  

Abra um novo terminal (ou um split do terminator) e rode `$ rosparam list`. Note que temos */sub_left/nome_topico* e */sub_right/nome_topico*. Dentro do launch, eles foram iniciados dentro do nó sendo, portanto, parâmetros locais. E foram devidamente importadas segundo os ROS_INFO_STREAM. Dentro do código, temos o `nh` "privado" e `nh.param<std::string>("nome_topico", nome_topico)`, portanto *nome_topico* é um parâmetro local e `nh` resolverá o *nome_topico* acrescentando o prefixo que indica a posição do nó dentro da hierarquia de nomes de ROS, ou seja, o próprio nome dado ao nó. Assim, o nome completo é */sub_left/nome_topico* ou */sub_right/nome_topico*.

Para deixar a questão da hierarquia de nomes mais clara, exploraremos o conceito de grupo no launch file. Crie outro arquivo XML em sub_rpm/launch chamado segundo.launch com o seguinte conteúdo:

```xml
<launch>
  <param name="param_global" value=""/>
  <group ns="robo1">    
    <param name="param_grupo" value=""/>
    <node pkg="gmr_intro" type="gmr_intro_node" name="gmr_pub_vel" output="screen">
    </node>
    <node pkg="sub_rpm" type="sub_rpm_node" name="sub_left" output="screen">
      <param name="nome_topico" value="/left_rpm"/>
    </node>

    <node pkg="sub_rpm" type="sub_rpm_node" name="sub_right" output="screen">
      <param name="nome_topico" value="/right_rpm"/>
    </node>
  </group>

  <group ns="robo2"> 
    <param name="param_grupo" value=""/>
    <node pkg="gmr_intro" type="gmr_intro_node" name="gmr_pub_vel" output="screen">
    </node>   
    <node pkg="sub_rpm" type="sub_rpm_node" name="sub_left" output="screen">
      <param name="nome_topico" value="/left_rpm"/>
    </node>

    <node pkg="sub_rpm" type="sub_rpm_node" name="sub_right" output="screen">
      <param name="nome_topico" value="/right_rpm"/>
    </node>
  </group>

</launch>
```

Note a existência de dois grupos `<group ns="robot1>` e `<group ns="robot2>`. Isso representa, por exemplo, a existência de dois robôs. Rodando novamente `$ rosparam list`:

```console
user@pc:~/ros_ws$ rosparam list
/axle_track
/gear_ratio
/param_global
/robo1/param_grupo
/robo1/sub_left/nome_topico
/robo1/sub_right/nome_topico
/robo2/param_grupo
/robo2/sub_left/nome_topico
/robo2/sub_right/nome_topico
/rosdistro
/roslaunch/uris/host_dell_inspiron_5480__35461
/rosversion
/run_id
/wheel_radius
```

Podemos ver que param_global, chamado fora de qualquer grupo ou nó, termina como `/param_global`. Já param_grupo, que é chamado dentro do grupo, termina como `/robo#/param_grupo`, em que # é o número do robô. E o nome_topico, que está dentro de um nó dentro de um grupo, acaba como `/robo#/sub_*/nome_topico, em que * é o lado considerado. Mas rodando `$ rostopic list`, temos que:

```console
user@pc:~/ros_ws$ rostopic list
/left_rpm
/right_rpm
/rosout
/rosout_agg
```

Apesar de não termos visto publicador ainda, note que o primeiro argumento na chamada do publicador em gmr_intro_node.cpp do pacote gmr_intro é o nome do tópico ao qual enviará as mensagens. E tal nome começa com "/", então é o nome final do tópico, independente da posição de gmr_intro_node dentro da hierarquia de nomes.  

```cpp
ros::Publisher pub_left_rpm = nh.advertise<std_msgs::Float32> ("/left_rpm",1);
```

Neste caso, os dois robôs estão publicando simultaneamente em /left_rpm e /right_rpm, o que exigiria mudança em gmr_intro_node.cpp para tornar os nomes locais. 

Com este tutorial, temos duas ferramentas fundamentais para acelerar o desenvolvimento da parte de software de um robô móvel: os parâmetros e os arquivos launch.


Contato: Akihiro (akihirohh@gmail.com)

Apoio da Fundação de Amparo à Pesquisa do Estado de São Paulo (FAPESP) através do processo nº 2018/10894-2. 
As opiniões, hipóteses e conclusões ou recomendações expressas neste material são de responsabilidade do(s) autor(es) e não necessariamente refletem a visão da FAPESP.