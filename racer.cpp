#include "warrior_18/racer.hpp"
#include <chrono>
#include <thread>

// Constructor
Racer::Racer(): Node ("robot_warrior")
{
    warrior_nick = "warrior_18";
    int cont = 0;

    // --- Inicializacion Variables ---
    battery = 100.0f;
    pos_x = 0.0f; pos_y = 0.0f; gamma = 0.0f;
    objetivo_x = 0.0f; objetivo_y = 0.0f;
    code = "-1"; 
    
    // Parametros Ajustables
    lin_vel_base = 0.7f;          // Un poco mas rapido
    collision_threshold_ = 0.45f; // Distancia critica para AVOID
    frontal_cone_rad_ = 60.0 * (M_PI / 180.0);
    avoid_angular_vel_ = 1.0f;
    umbral_distancia_meta = 0.4f; // Radio para considerar "tocado"
    freq_ejecucion = 10.0f;        // Hz

    // PID (Ajustado)
    pid_kp = 0.5f;   
    pid_ki = 0.008f;  
    pid_kd = 2.4f;    // KD bajado, 1.9 es demasiado freno
    prev_error = 0.0f;
    integral_error = 0.0f;

    // VFF Pesos
    peso_atraccion = 0.8f;
    peso_repulsion_max = 1.1f; // Repulsion siempre gana a atraccion si esta cerca

    current_state_ = States::DECIDIR_OBJETIVO;
    objetivo_actual = TipoObjetivo::NINGUNO;

    fuerza_repulsion = {0.0f, 0.0f};

    // --- Registro en el Servidor ---
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request->username = warrior_nick;
    
    while(!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio de registro...");
    }
    
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;

            if (code == "-1") {
                warrior_nick = "warrior_18_" + std::to_string(++cont);
                request->username = warrior_nick;
                RCLCPP_WARN(this->get_logger(), "Nick ocupado, probando: %s", warrior_nick.c_str());
            } else {   
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 10 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 10 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 1, std::bind(&Racer::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 1, std::bind(&Racer::process_scene_info, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "RACER Registrado. ID: %s", code.c_str());          
            }
        }
    }
}

Racer::~Racer() {
     RCLCPP_ERROR(this->get_logger(), "Racer Destruido");
}

// --- CALLBACK LASER (Fuerza Repulsiva) ---
void Racer::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    fuerza_repulsion = {0.0f, 0.0f};
    float dist_minima_detectada = 5.0f;
    int rayos_activos = 0;
    float rango_seguridad = 2.0f; // Solo importan obstaculos a menos de 2m

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {   
        float d = msg->ranges[i];
        if (d < rango_seguridad && d > 0.05f) 
        {
            if (d < dist_minima_detectada) dist_minima_detectada = d;
            float ang = msg->angle_min + i * msg->angle_increment;
            
            // OPTIMIZACION: Ponderar cada rayo por su cercania inversa (1/d)
            // Los objetos mas cercanos empujan con mas fuerza
            float peso_rayo = (rango_seguridad - d); 
            
            fuerza_repulsion.f_x += -cos(ang) * peso_rayo;
            fuerza_repulsion.f_y += -sin(ang) * peso_rayo;
            rayos_activos++;
        }
    }

    if (rayos_activos > 0)
    {
        // Normalizamos vector direccion
        float modulo = sqrt(pow(fuerza_repulsion.f_x, 2) + pow(fuerza_repulsion.f_y, 2));
        if (modulo > 0.001f) {
            fuerza_repulsion.f_x /= modulo;
            fuerza_repulsion.f_y /= modulo;
        }

        // Intensidad global basada en el objeto mas cercano
        float intensidad = (rango_seguridad - dist_minima_detectada) / rango_seguridad; // 0 a 1
        intensidad = std::max(0.0f, std::min(1.0f, intensidad));

        fuerza_repulsion.f_x *= (intensidad * peso_repulsion_max);
        fuerza_repulsion.f_y *= (intensidad * peso_repulsion_max);
    }
    
    // Auxiliar para FSM (Avoid State)
    front_dist_ = GetMinInSector(msg, frontal_cone_rad_);
}

// --- CALLBACK ESCENA ---
void Racer::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    std::string errs;
    
    if(Json::parseFromStream(reader, jsonStream, &JsonSceneData, &errs)) {
        battery = JsonSceneData["Battery_Level"].asFloat();
        pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
        pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
        gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

        // Limpiamos y llenamos de nuevo (Reactivo puro)
        chargers_pos_array.clear(); 
        coins_pos_array.clear();

        const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
        for (const Json::Value &charger : chargers_pos) {
            std::vector<float> data;
            for (const Json::Value &val : charger) data.push_back(val.asFloat());
            chargers_pos_array.push_back(data);
        }

        const Json::Value &coins_pos = JsonSceneData["FOV"]["Coins_Positions"];
        for (const Json::Value &coin : coins_pos) {
            std::vector<float> data;
            for (const Json::Value &val : coin) data.push_back(val.asFloat());
            coins_pos_array.push_back(data);
        }
    }
}

// --- LOGICA DEL ROBOT ---

// Funcion de coste: Elige la moneda que esta cerca Y de frente
float Racer::calcula_coste(const std::vector<float>& pos)
{
    float dx = pos[0] - pos_x;
    float dy = pos[1] - pos_y;
    float dist = sqrt(dx*dx + dy*dy);
    
    // Calculo del angulo relativo hacia esa moneda
    float angulo_global = atan2(dy, dx);
    float angulo_local = angulo_global - gamma;
    while (angulo_local > M_PI) angulo_local -= 2*M_PI;
    while (angulo_local < -M_PI) angulo_local += 2*M_PI;

    // Coste = Distancia + Penalizacion por tener que girar
    // Si la moneda esta detras (pi radianes), añadimos coste virtual "metros"
    float coste = dist + (std::abs(angulo_local) * 1.5); 
    return coste;
}

// Logica de Histeresis para la bateria
bool Racer::necesito_cargar()
{
    // 1. MEMORIA (HISTÉRESIS): 
    // Si mi objetivo ACTUAL ya es la batería, NO cambio de opinión
    // hasta que esté cargado (por ejemplo al 90% o 95%).
    // Esto evita que se de la vuelta a mitad de camino.
    if (objetivo_actual == TipoObjetivo::BATERIA)
    {
        if (battery < 95.0f) return true; // Sigo necesitando cargar
        return false; // Ya estoy lleno, suelto el cargador
    }

    // 2. Lógica normal para cuando estoy buscando monedas
    
    // Emergencia absoluta
    if (battery < 25.0f) return true;

    // Calculo dinámico
    if (!chargers_pos_array.empty())
    {
        float min_dist_charger = 1000.0f;
        for(auto& c : chargers_pos_array)
        {
            float d = sqrt(pow(c[0]-pos_x, 2) + pow(c[1]-pos_y, 2));
            if(d < min_dist_charger) min_dist_charger = d;
        }
        
        // Factor de seguridad
        // bateria por TICK = 0.05*v_max*coef.seguridad + 0.1.¡; coef.seguridad = 1.7; v_max=1;
        //bateria camino = bateria por TICK * tiempo camino = 0.185 * distancia
        float bateria_necesaria = min_dist_charger*(0.185); 
        if (battery < bateria_necesaria) return true;
    }

    return false; 
}

void Racer::decidir_objetivo()
{
    // 1. Prioridad: Bateria
    if (necesito_cargar() && !chargers_pos_array.empty())
    {
        objetivo_actual = TipoObjetivo::BATERIA;
        
        // Buscar cargador mas cercano
        float min_dist = 10000.0f;
        for (const auto &c : chargers_pos_array)
        {
            float dist = sqrt(pow(c[0] - pos_x, 2) + pow(c[1] - pos_y, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                objetivo_x = c[0];
                objetivo_y = c[1];
            }
        }
        RCLCPP_INFO(this->get_logger(), "OBJETIVO: CARGADOR (Bat: %.1f)", battery);
        objetivo_decidido = true;
        return;
    }

    // 2. Prioridad: Monedas
    if (!coins_pos_array.empty()) {
        objetivo_actual = TipoObjetivo::MONEDA;
        float min_coste = 10000.0f;
        
        for (const auto &coin : coins_pos_array) {
            float coste = calcula_coste(coin);
            if (coste < min_coste) {
                min_coste = coste;
                objetivo_x = coin[0];
                objetivo_y = coin[1];
            }
        }
        objetivo_decidido = true;
    } else {
        // No veo nada, me quedo quieto o giro para buscar (pendiente)
        objetivo_decidido = false;
        objetivo_actual = TipoObjetivo::NINGUNO;
    }
}

Force Racer::calc_fuerza_atraccion()
{
    Force f = {0.0f, 0.0f};
    if (!objetivo_decidido) return f;

    float dx = objetivo_x - pos_x;
    float dy = objetivo_y - pos_y;
    
    // Transformar a Local
    float dx_local = dx * cos(gamma) + dy * sin(gamma);
    float dy_local = -dx * sin(gamma) + dy * cos(gamma);

    float mod = sqrt(dx_local*dx_local + dy_local*dy_local);
    if (mod > 0.05f) {
        f.f_x = (dx_local / mod) * peso_atraccion;
        f.f_y = (dy_local / mod) * peso_atraccion;
    }
    return f;
}

// UNIFICA EL CONTROL PID Y VFF
void Racer::control_navegacion()
{
    // 1. Calcular Vectores
    Force f_att = calc_fuerza_atraccion();
    
    // Suma VFF
    float fx_total = f_att.f_x + fuerza_repulsion.f_x;
    float fy_total = f_att.f_y + fuerza_repulsion.f_y;

    // 2. Calcular Angulo Deseado
    float target_angle = atan2(fy_total, fx_total);

    // 3. PID Angular
    float error = target_angle; 
    
    if (std::abs(error) > 0.1) integral_error += error;
    else integral_error = 0.0f;

    float derivative = error - prev_error;
    float angular_cmd = (pid_kp * error) + (pid_ki * integral_error/freq_ejecucion) + (pid_kd * derivative);
    
    prev_error = error;
    
    // Clamp angular
    angular_cmd = std::max(-2.0f, std::min(2.0f, angular_cmd));

    // 4. Control Lineal Inteligente
    float linear_cmd = lin_vel_base;
    
    // Si el error angular es grande, frena para girar
    //if (std::abs(error) > 0.5f) linear_cmd *= 0.2f;
    //else if (std::abs(error) > 0.2f) linear_cmd *= 0.6f;

    // Si hay obstaculo cerca (aunque la repulsion nos desvie), reduce velocidad
    if (front_dist_ < 1.5f) linear_cmd *= 0.5f;

    // 5. Publicar
    PublishVelocity(linear_cmd, angular_cmd);
}

// --- MAQUINA DE ESTADOS PRINCIPAL ---
void Racer::FSM_Control_Loop()
{
    RCLCPP_INFO(this->get_logger(), "BATERIA ACTUAL: %.2f", battery);
    switch (current_state_)
    {
    case States::DECIDIR_OBJETIVO:
        decidir_objetivo(); // Elige entre Bateria o Moneda segun necesidad
        
        if (objetivo_decidido) {
            // Reset PID
            prev_error = 0.0f; integral_error = 0.0f;
            current_state_ = States::NAVEGANDO;
        } else {
            // Girar para buscar (Search Mode)
            //PublishVelocity(0.0, 0.5); 
        }
        break;

    case States::NAVEGANDO: // Sirve para Moneda y Cargador
        decidir_objetivo();
        control_navegacion();

        // Chequeo de llegada
        if (he_llegado_a_objetivo())
        {
            PublishVelocity(0.0, 0.0);
            
            if (objetivo_actual == TipoObjetivo::BATERIA)
            {
                current_state_ = States::ESTOY_CARGANDO;
            } else {
                current_state_ = States::DECIDIR_OBJETIVO; // A por la siguiente
            }
        }
        
        // Chequeo de Emergencia
        if (front_dist_ < collision_threshold_)
        {
            current_state_ = States::AVOID;
        }
        break;

    case States::ESTOY_CARGANDO:
    {
        // Se queda quieto hasta que necesites_cargar() devuelva false (al llegar al 95%)
        float w_cargando = orientar_hacia_moneda();
        PublishVelocity(0.0, w_cargando*((battery) / 95)); // Gira despacio mientras carga
        
        if (!necesito_cargar()) {
            RCLCPP_INFO(this->get_logger(), "Carga Completa. A por monedas.");
            current_state_ = States::DECIDIR_OBJETIVO;
        }
        break;
    }
        

    case States::AVOID:
        PublishVelocity(0.0, avoid_angular_vel_);
        
        if (front_dist_ > collision_threshold_ * 2.0f) {
             // Si estábamos yendo al cargador, mantenemos ese estado para no recalcular mal
             if (objetivo_actual == TipoObjetivo::BATERIA) {
                 current_state_ = States::NAVEGANDO;
             } else {
                 current_state_ = States::DECIDIR_OBJETIVO;
             }
        }
        break;
    }
}

// --- AUXILIARES ---

bool Racer::he_llegado_a_objetivo()
{
    float dist = sqrt(pow(objetivo_x - pos_x, 2) + pow(objetivo_y - pos_y, 2));
    if (dist < umbral_distancia_meta) {
        if (objetivo_actual == TipoObjetivo::MONEDA) {
            RCLCPP_INFO(this->get_logger(), "Moneda conseguida!");
            // Nota: No borramos del array manualmente, esperamos al update del servidor
        }
        return true;
    }
    return false;
}

float Racer::GetMinInSector(const sensor_msgs::msg::LaserScan::SharedPtr msg, float angle_width_rad)
{
    float min_val = 10.0f;
    // Implementacion simplificada correcta
    int center_idx = (msg->angle_max - msg->angle_min) / msg->angle_increment / 2;
    int range_width = (angle_width_rad / msg->angle_increment) / 2;

    for (int i = center_idx - range_width; i < center_idx + range_width; i++) {
        if (i >= 0 && i < (int)msg->ranges.size()) {
             float d = msg->ranges[i];
             if (d > 0.05 && d < min_val) min_val = d;
        }
    }
    return min_val;
}

float Racer::orientar_hacia_moneda()
{
    if (!coins_pos_array.empty()) {
        float min_coste = 10000.0f;
        
        for (const auto &coin : coins_pos_array) {
            float coste = calcula_coste(coin);
            if (coste < min_coste) {
                min_coste = coste;
                moneda_siguiente_x= coin[0];
                moneda_siguiente_y = coin[1];
            }
        }
    }
    return atan2(moneda_siguiente_y - pos_y, moneda_siguiente_x - pos_x) - gamma;
}

void Racer::PublishVelocity(float vel_lineal, float vel_angular)
{
    rosgame_msgs::msg::RosgameTwist cmd;
    cmd.code = this->code;
    
    // Escalado dinamico si gira rapido (para no derrapar)
    float linear_scale = 1.0f;
    if (std::abs(vel_angular) > 0.5f) linear_scale = 0.4f;

    cmd.vel.linear.x = vel_lineal * linear_scale;
    cmd.vel.angular.z = vel_angular;
    
    v = cmd.vel.linear.x; 
    w = cmd.vel.angular.z;
    pub1_->publish(cmd);
}

int main(int argc, char **argv)
{
     // ros2 launch warrior_18 racer_launch.xml
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Racer>();
    std::this_thread::sleep_for(std::chrono::milliseconds(1800));
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->FSM_Control_Loop();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}