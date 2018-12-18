# fsg_opt_data_storage

Generate/Load data file for optimization.

# Executable file

- generate_2d

    Generate 2d data

    There are several data model and noise model (not yet. will be there near future.)

    ```bash
    rosrun fgs_opt_data_storage generate_2d PATH_TO_THIS_PACKAGE/config/curve2d.yaml
    ```

# fgs_opt_data_storage_load_sample

    A sample to use loading library.

    ```bash
    rosrun fgs_opt_data_storage fgs_opt_data_storage_load_sample /tmp/data_2d.yaml
    ```
