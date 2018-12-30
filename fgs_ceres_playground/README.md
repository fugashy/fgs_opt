# fgs_ceres_playground

My ceres playground.

# Executable files

- fgs_ceres_playground_hello_world

    Minimize f(x) = (1/2) * (10.0 - x)^2

    Support 3 types of derivatives: auto, numerical and analytical

    ```bash
    rosrun fgs_ceres_playground fgs_ceres_playground_hello_world auto
    ```

- fgs_ceres_playground_fitting

    Simple fitting samples

    ```bash
    rosrun fgs_opt_data_storage generate_data_2d /path/to/this/pkg/config/curve2d.yaml
    rosrun fgs_ceres_playground fgs_ceres_playground_fitting /tmp/data_curve2d.yaml curve2d
    ```
