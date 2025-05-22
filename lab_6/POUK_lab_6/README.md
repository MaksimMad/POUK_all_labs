## Установка ros1_bridge

```sudo apt-get install ros-foxy-ros1-bridge```


## Сборка
из корневой папки:

```source /opt/ros/foxy/setup.bash```

```colcon build```

## Запуск

1-ый терминал

Запускаем navigation 

(команды выполняются при нахождении в директории рабочего пространства)  
0. Скачиваем проект:
```bash
git clone <адрес репозитария> src
```
или обновляем исходные файлы (если проект уже скачан):
```bash
cd src  
git pull  
cd ..  
```
В папке `src` должны появиться проекты `navigation`
1. Сборка проекта (из рабочей директории):
```bash
catkin_make
```
2. Не забываем в каждом терминале, где планируем запускать бинарные файлы проекта, инициализировать рабочее пространство:
```bash
source devel/setup.bash
```
3. Запуск модели робота, модуля навигации и управлением движением робота:  
```bash
roslaunch navigation navi.launch
```
Должно открыться окно симулятора Stage с роботом в мире с препятствиями. Должны запуститься модули управления и навигации, а также открыться окно интерфейса - программы rviz



2-ой терминал

```source /opt/ros/noetic/setup.bash```

```source /opt/ros/foxy/setup.bash```

```source install/setup.bash```

```ros2 run ros1_bridge dynamic_bridge --bridge-config-file $(ros2 pkg prefix patrol_bot)/share/patrol_bot/config/bridge.yaml```


3-ий терминал

```source /opt/ros/foxy/setup.bash```

```source install/setup.bash```

```ros2 run patrol_bot patrol_bot_node ```
