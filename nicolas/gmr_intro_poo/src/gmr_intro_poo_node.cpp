/* =========== */
/*  Libraries  */
/* =========== */
#include "../include/gmr_intro_poo/gmr_intro_poo.hpp"

/* =========== */
/*  Functions  */
/* =========== */

/* ====== */
/*  Main  */
/* ====== */
int main(int argc, char **argv)
{
    // Inicialização da ROS no contexto deste nó
    ros::init(argc, argv, "gmr_intro_poo_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(50);  //Loop rate in Hz

    // Criação de uma instância da classe RobotClass em que
    // um ponteiro a ros::NodeHandle é passado como argumento
    RobotClass robot(&nh);

    while(ros::ok())
    {
        robot.checkToggleRobot();
        robot.calculateOdom();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
