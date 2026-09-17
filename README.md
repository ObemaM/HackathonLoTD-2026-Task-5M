# Metro Obstacle Detector — Task 5M

Геометрический baseline системы обнаружения посторонних объектов в габарите поезда по
данным 3D-лидара. Проект рассчитан на Ubuntu 22.04, ROS 2 Humble и запуск в Docker.

> Текущая версия — рабочий геометрический baseline с автоматической оценкой своей
> пары рельсов и изогнутым коридором. Это всё ещё экспериментальная система:
> параметры второго лидара и подавление всей штатной инфраструктуры требуют калибровки.

## Как это работает

```text
ros2 bag play
      │ PointCloud2
      ▼
detector_node
      ├─ удаляет NaN и конечные точки (0,0,0)
      ├─ приводит оси к X-forward, Y-left, Z-up
      ├─ уменьшает облако voxel-фильтром
      ├─ ищет свою пару рельсов в последовательных поперечных срезах
      ├─ продолжает локальную центральную линию от ближней зоны наружу
      ├─ обрезает доверенный путь при неоднозначной стрелке
      ├─ строит вдоль неё габарит 2.1 x 3.0 м
      ├─ маскирует рельсы и устойчивые низкие продольные конструкции
      ├─ объединяет остатки в 3D-кластеры
      └─ подтверждает ближайший кластер на нескольких кадрах
      ▼
detected + distance + debug clouds + RViz markers
```

Подробности: [архитектура](docs/architecture.md) и [план разработки](docs/development_plan.md).

## Структура репозитория

```text
.
├── Dockerfile
├── docker/                 # entrypoint контейнера
├── ros2_ws/src/
│   └── metro_obstacle_detector/
│       ├── src/            # C++ ROS-node
│       ├── include/        # преобразование систем координат
│       ├── config/         # параметры для normal/obstacle bags
│       ├── launch/         # единая команда запуска
│       ├── rviz/           # готовая RViz-конфигурация
│       └── test/           # C++ unit tests
├── tools/                  # офлайн-анализ bags на Python
├── experiments/            # сценарии и будущие результаты
├── docs/                   # архитектура и roadmap
└── scripts/                # сборка и тесты в WSL
```

Большие bag-файлы в репозиторий не добавляются.

## Требования

- Ubuntu 22.04;
- ROS 2 Humble;
- `libpcl-dev`;
- `ros-humble-pcl-conversions`;
- `ros-humble-tf2-ros`;
- `python3-colcon-common-extensions`.

На текущем компьютере всё необходимое уже установлено в WSL-дистрибутиве
`Ubuntu-22.04`. Дефолтный `Ubuntu` — это 24.04 и для этого проекта не используется.

## Сборка в WSL

Из PowerShell открыть правильный дистрибутив:

```powershell
wsl -d Ubuntu-22.04
```

В WSL:

```bash
cd /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M
chmod +x scripts/*.sh docker/entrypoint.sh
./scripts/build_wsl.sh
```

Или вручную:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Запуск на обычных moving bags

Потребуются три терминала WSL.

### Терминал 1 — detector

```bash
source /opt/ros/humble/setup.bash
source /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M/ros2_ws/install/setup.bash

ros2 launch metro_obstacle_detector detector.launch.py \
  input_topic:=/lidar_points \
  use_sim_time:=true
```

### Терминал 2 — bag

```bash
source /opt/ros/humble/setup.bash

ros2 bag play \
  ~/HackathonLoDT/data/for_hackathon/roundT_doubleT \
  --clock --loop
```

### Терминал 3 — RViz

```bash
source /opt/ros/humble/setup.bash
source /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M/ros2_ws/install/setup.bash

rviz2 -d \
  /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M/ros2_ws/install/metro_obstacle_detector/share/metro_obstacle_detector/rviz/detector.rviz
```

Можно запустить RViz вместе с detector:

```bash
ros2 launch metro_obstacle_detector detector.launch.py rviz:=true
```

## Запуск `doubleT_obstacle`

У этого bag другой topic, frame и положение лидара. Для него подготовлен отдельный
начальный конфиг:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch metro_obstacle_detector detector.launch.py \
  config:=$(ros2 pkg prefix metro_obstacle_detector)/share/metro_obstacle_detector/config/doubleT_obstacle.yaml \
  input_topic:=/sensing/lidar/hesai128/pointcloud \
  use_sim_time:=true \
  rviz:=true
```

В другом терминале:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play \
  ~/HackathonLoDT/data/for_hackathon/doubleT_obstacle \
  --clock --loop
```

Значения `corridor.center_lateral_m` и `corridor.bottom_z_m` в obstacle-конфиге —
предварительная визуальная оценка. Их нужно откалибровать в RViz и подтвердить
экспериментом.

## Выходные topics

| Topic | Тип | Смысл |
|---|---|---|
| `/obstacle_detector/detected` | `std_msgs/Bool` | Есть ли допустимый кластер |
| `/obstacle_detector/distance_m` | `std_msgs/Float32` | Расстояние до ближайшего, `-1` если ничего нет |
| `/obstacle_detector/processing_time_ms` | `std_msgs/Float32` | Время обработки кадра |
| `/obstacle_detector/track_valid` | `std_msgs/Bool` | Уверенно ли найдена собственная колея |
| `/obstacle_detector/route_ambiguous` | `std_msgs/Bool` | Видны две расходящиеся допустимые ветви, а маршрут неизвестен |
| `/obstacle_detector/debug/filtered` | `PointCloud2` | Валидное прореженное облако |
| `/obstacle_detector/debug/candidates` | `PointCloud2` | Точки внутри текущего габарита |
| `/obstacle_detector/markers` | `MarkerArray` | Коридор, bounding boxes и подписи |

Проверить ответы без RViz:

```bash
ros2 topic echo /obstacle_detector/detected
ros2 topic echo /obstacle_detector/distance_m
ros2 topic hz /obstacle_detector/detected
```

## Параметры

Основные параметры находятся в
`ros2_ws/src/metro_obstacle_detector/config/default.yaml`.

- `forward_axis`, `lateral_axis`, `vertical_axis` — соответствие осей исходного лидара;
- `detector_offset_xyz` — перенос начала координат;
- `voxel_leaf_size_m` — степень прореживания;
- `corridor.*` — ручное положение габарита;
- `track.*` — поиск пары рельсов и сглаживание центральной линии;
- `rail_mask.*` — узкая маска двух найденных рельсов;
- `longitudinal_mask.*` — низкие продольные конструкции, устойчивые на многих срезах;
- `confirmation.*` — число кадров для подтверждения/снятия тревоги;
- `clustering.*` — параметры объединения и фильтрации кластеров;
- `publish_debug_clouds` — выключить тяжёлые debug topics при benchmark.

После изменения YAML при сборке без `--symlink-install` нужно повторить `colcon build`.

## Анализ bag без запуска detector

```bash
source /opt/ros/humble/setup.bash

python3 tools/analyze_bag.py \
  ~/HackathonLoDT/data/for_hackathon/roundT_doubleT \
  --sample-every 10 \
  --forward-axis=-y
```

Скрипт работает только на чтение и показывает topic, frame, частоту, количество
ненулевых точек и фактическую дальность.

## Тесты

```bash
./scripts/test_wsl.sh
```

Первый unit test проверяет преобразование осей и обратимость систем координат.

## Docker

Собрать из корня репозитория:

```bash
docker build -t metro-obstacle-detector:dev .
```

Запустить detector в Linux/WSL с host networking:

```bash
docker run --rm --net=host \
  metro-obstacle-detector:dev \
  ros2 launch metro_obstacle_detector detector.launch.py \
  input_topic:=/lidar_points
```

Bag можно проигрывать на хосте. Через `--net=host` DDS-трафик ROS 2 будет доступен
контейнеру. Если локальная Docker Desktop-конфигурация не пропускает DDS, на время
разработки запускайте detector и bag внутри одного WSL-дистрибутива; финальный стенд
организаторов — обычный Linux.

## Что означают линии в RViz

- голубая — оценённая центральная линия своего пути;
- жёлтые — найденная пара ходовых рельсов;
- зелёные — границы динамического габарита;
- фиолетовые — низкие продольные линии штатной инфраструктуры, исключённые из кандидатов;
- оранжевый шар — конец доверенного коридора перед неоднозначной стрелкой;
- оранжевые/красные рамки — оставшиеся геометрические кандидаты.

Если `/obstacle_detector/track_valid=false`, алгоритм не доверяет оценке рельсов и
возвращается к прямому ручному коридору. Это намеренный безопасный fallback.
Если `/obstacle_detector/route_ambiguous=true`, найденная часть пути остаётся валидной,
но коридор заканчивается перед местом, где без маршрутных данных нельзя честно выбрать
левую или правую ветвь.

## Известные ограничения baseline

1. Для `doubleT_obstacle` автоматический поиск рельсов пока выключен: у него другое
   положение лидара, высотный срез необходимо откалибровать отдельно.
2. Стрелка обнаруживается как неоднозначность и ограничивает дальность доверенного
   коридора. Для выбора ветви заранее всё ещё нужны маршрутные данные или полноценный
   банк нескольких временных гипотез.
3. Некоторые низкие штатные элементы всё ещё могут оставаться кандидатами. Нельзя
   просто поднять порог высоты: по условиям требуется обнаруживать объект высотой
   около 10 см и оборванный кабель.
4. Кластеризация пока имеет фиксированный tolerance и не адаптируется к дальности.
5. Три последовательных кадра подтверждают устойчивость, но это ещё не полноценный
   трекер с компенсацией движения поезда.
6. Нет эталонной карты чистого проезда и генератора синтетических препятствий.
7. Ответ `detected=true` означает подтверждённый геометрический кандидат. До проверки
   на размеченной синтетике его нельзя считать доказанным препятствием.

Эти ограничения сделаны явными специально: текущий проект даёт работающую платформу,
в которой следующие гипотезы можно измерять и заменять по одной.
