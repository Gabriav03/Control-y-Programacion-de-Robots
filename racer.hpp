#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"
#include "rclcpp/time.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

// Estados de la FSM
enum class States {
    DECIDIR_OBJETIVO,
    NAVEGANDO,       // Unifica IR_MONEDA e IR_CARGADOR
    ESTOY_CARGANDO,
    AVOID
};

// Tipos de fuerza
enum class TipoFuerza {
    ATRACCION,
    REPULSION
};

// Objetivos
enum class TipoObjetivo {
    MONEDA,
    BATERIA,
    NINGUNO
};

// Estructura Vectorial
struct Force {
    float f_x;
    float f_y;
};

class Racer : public rclcpp::Node
{
public:
    Racer();
    ~Racer();
    void FSM_Control_Loop();

private:
    // Callbacks
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);

    // Funciones principales
    void decidir_objetivo();
    void control_navegacion(); // Unifica PID y VFF
    void PublishVelocity(float vel_lineal, float vel_angular);
    
    // Auxiliares
    Force calc_fuerza_atraccion();
    float GetMinInSector(const sensor_msgs::msg::LaserScan::SharedPtr msg, float angle_width_rad);
    bool he_llegado_a_objetivo();
    bool necesito_cargar(); // Logica de umbral bateria
    float calcula_coste(const std::vector<float>& pos); // Funcion de coste corregida
    float orientar_hacia_moneda(); // Devuelve velocidad angular para orientar hacia la moneda mas cercana  

    // PUBLICADORES / SUSCRIPTORES
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub1_;
    rclcpp::Publisher<rosgame_msgs::msg::RosgamePoint>::SharedPtr pub2_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
    
    rclcpp::Client<rosgame_bridge::srv::RosgameRegister>::SharedPtr client_;

    // Variables Identidad
    std::string warrior_nick;
    std::string code;
    
    States current_state_;
    TipoObjetivo objetivo_actual;

    // Estado del Robot
    float battery;
    float pos_x, pos_y, gamma;
    float objetivo_x, objetivo_y,moneda_siguiente_x,moneda_siguiente_y;
    
    // Variables Control y Sensores
    float frontal_cone_rad_;
    float avoid_angular_vel_;
    float umbral_distancia_meta; // Distancia para considerar "Llegada"
    bool objetivo_decidido;
    float v, w;
    
    // PID
    float pid_kp, pid_ki, pid_kd;
    float prev_error, integral_error;
    
    // VFF Parametros
    float peso_atraccion; 
    float peso_repulsion_max;
    float lin_vel_base;
    float freq_ejecucion;

    // Vectores
    Force fuerza_repulsion;
    
    // Percepcion
    float front_dist_;      
    float collision_threshold_;

    // Escena
    std::vector<std::vector<float>> chargers_pos_array;
    std::vector<std::vector<float>> coins_pos_array;
};