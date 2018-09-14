# DDPG Avoid

## 本程式使用的是pyhton3,如果使用pyhon請注意記得改
* 各執行檔（ros_ddpg內）
    ```bash

    #!/usr/bin/env python3 -> #!/usr/bin/env python

    ```

## 如果有使用新的訓練參數(名字不一樣)時,記得改
* _**config/checkpoint**_ 
    
    ```bash
    model_checkpoint_path: "新檔名"
    all_model_checkpoint_paths: "新檔名"

    ```
* _**ros_ddpg/lib/DDPG.py**_

    ```bash
    FILENAME = rospkg.RosPack().get_path('ddpg')+'/config/新檔名'
    ```
