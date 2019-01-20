# fgs_g2o_playground

My g2o playground

# Executable files

- fgs_g2o_playground_fitting

  Minimize least squared mean error.

  ```bash
  rosrun fgs_opt_data_storage generate_data_2d /path/to/this/pkg/config/curve2d.yaml
  rosrun fgs_g2o_playground fgs_g2o_playground_fitting /tmp/data_curve2d.yaml curve
  ```
