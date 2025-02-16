#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LaserBrakeNode : public rclcpp::Node
{
public:
    LaserBrakeNode() : Node("laser_brake_node")
    {
        // Suscripción al topic /scan
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserBrakeNode::laserCallback, this, std::placeholders::_1));

        // Suscripción al topic cmd_vel para obtener la velocidad del vehículo
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&LaserBrakeNode::cmdVelCallback, this, std::placeholders::_1));

        // Publicación al topic cmd_vel_break
        brake_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_break", 10);

        // Valor umbral de TTC para iniciar el frenado (en segundos)
        ttc_threshold_ = 1.0;
        current_velocity_ = 0.0;  // Inicializamos la velocidad en 0
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
    // Encontrar la distancia mínima frente al vehículo
        float min_distance = std::numeric_limits<float>::infinity();
        int start_index = (msg->ranges.size() / 2) - 10;  // Ángulo cercano a 0°
        int end_index = (msg->ranges.size() / 2) + 10;

        RCLCPP_INFO(this->get_logger(), "Objetos frente al carro:");

        // Variable para guardar la velocidad ajustada por el ángulo
        float velocity_projection = 0.0;

        for (int i = start_index; i <= end_index; ++i)
        {
            float distance = msg->ranges[i];

            // Solo consideramos distancias válidas (descartamos Inf o NaN)
            if (distance > 0.0 && distance < 10.0) // Rango de interés: >0 y <10 metros
            {
                float angle = msg->angle_min + i * msg->angle_increment;

                // Si encontramos una nueva distancia mínima
                if (distance < min_distance)
                {
                    min_distance = distance;
                }

                RCLCPP_INFO(this->get_logger(), "Distancia a %.2f°: %.2f m", 
                            angle * 180.0 / M_PI, distance);
            }
        }

         // Si se detecta algún objeto dentro del rango
        if (min_distance < std::numeric_limits<float>::infinity())
        {
            // Solo calcular TTC si la velocidad es no nula
            if (current_velocity_ != 0.0)
            {
                // Calcular la velocidad proyectada en la dirección del láser
                velocity_projection = std::max(-current_velocity_ * std::cos(msg->angle_min), 0.0f);

                // Cálculo del TTC
                if (velocity_projection != 0.0f)
                {
                    float ttc = min_distance / velocity_projection;

                    // Imprimir el TTC
                    RCLCPP_INFO(this->get_logger(), "TTC calculado: %.2f s", ttc);

                    // Comando de frenado
                    geometry_msgs::msg::Twist brake_cmd;

                    if (ttc < ttc_threshold_)
                    {
                        brake_cmd.linear.x = -1.0;  // Frenado de emergencia
                        RCLCPP_WARN(this->get_logger(), "TTC = %.2f s. Frenando de emergencia.", ttc);
                        // Publicar el comando de frenado
                        brake_publisher_->publish(brake_cmd);
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Velocidad proyectada es 0, no se calcula el TTC.");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Velocidad es 0, no se calcula el TTC.");
            }
        }
}

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extraer la velocidad lineal en el eje X (hacia adelante)
        current_velocity_ = msg->linear.x;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr brake_publisher_;

    float current_velocity_;
    float ttc_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserBrakeNode>());
    rclcpp::shutdown();
    return 0;
}


