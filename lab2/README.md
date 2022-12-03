# Lab 2

- [Lab 2](#lab-2)
  - [The Construct](#the-construct)
  - [Workspace](#workspace)
  - [Package](#package)
  - [Nodes](#nodes)
  - [Topics](#topics)
  - [Services](#services)
  - [Actions](#actions)
  - [Źródła](#źródła)

## The Construct

Tworzymy nowy pusty ROSject na platformie [The Construct](https://www.theconstructsim.com/), wersja ROS Noetic.

## Workspace

1. `mkdir catkin_ws` (lub o dowolnej innej nazwie)
2. `cd catkin_ws`
3. `mkdir src` (musi istnieć, żeby catkin zadziałał)
4. `catkin_make` - inicjalizuje pusty workspace
5. `source ./devel/setup.bash` - inicjalizujemy środowisko naszego workspacea.

**Uwaga:** Przed uruchomieniem catkina, musimy mieć zainicjalizowane środowisko ROSowe w nasym terminalu za pomocą `source /opt/ros/noetic/setup.bash`. The Construct robi to za nas. Jeśli mamy ROSa na własnym komputerze, możemy dodać tą linię do `.bashrc`.

**Czytaj więcej:** [Creating a workspace for catkin.](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## Package

```sh
  catkin_create_pkg basics rospy std_msgs actionlib_msgs message_generation
```

- `basics` - nazwa nowej paczki,
- `rospy` - dependencje do Pythona,
- `std_msgs` - dependencja do standardowych wiadomości ROSowych,
- `actionlib_msgs` - dependencja do akcji,
- `message_generation` - dependencja do generowania nowych wiadomości i serwisów.

W katalogu `catkin_ws/src/basics` pojawią się pliki:

- `package.xml` - plik z informacjami o paczce (np. nazwa, wersja, autorzy, maintainerzy, opis, dependencje z innych paczek),
- `CMakeLists.txt` - plik opisujący budowanie/instalowanie paczki.

**Uwaga:** jeśli dependencje w `package.xml` są błędne, a w `CMakeLists.txt` poprawne, to paczką zbuduję się u nas poprawnie, ale pojawią się problemy podczas publikowania paczki.

## Nodes

<img src="https://docs.ros.org/en/foxy/_images/Nodes-TopicandService.gif" width=500/>

Node, czyli po prostu program, który jest widziany przez ROSa i może korzystać z jego dobrodziejstw.

1. Tworzymy plik `node.py` w katalogu `catkin_ws/src/basics/src`:

   ```py
   #!/usr/bin/env python3
   import rospy

   def main():
       rospy.init_node("node")
       rospy.loginfo("Hello World!")
       rospy.spin()

   if __name__ == "__main__":
       main()
   ```

2. Pamiętamy o [shebangu (#!)](<https://en.wikipedia.org/wiki/Shebang_(Unix)>).
3. Ustawiamy uprawnienia do wykonywania pliku:
   ```sh
   chmod +x node.py
   ```
4. Uruchamiamy node:
   ```sh
   rosrun basics node.py
   ```
5. Możemy skorzystać z poleceń:
   ```sh
   rosnode list
   ```
   ```sh
   rosnode info /node
   ```
   aby dowiedzieć się więcej o uruchomionych w tle nodeach.

**Uwaga:** Żeby node mógł poprawnie działać, w tle musi zostać uruchomiony _master_ za pomocą polecania `roscore`. The Construct robi to za nas i od razu działa on w tle.

**Czytaj więcej:** [ROS Nodes](http://wiki.ros.org/Nodes).

## Topics

<img src="https://docs.ros.org/en/foxy/_images/Topic-SinglePublisherandSingleSubscriber.gif" width=500/>

Topic, czyli sposób na przekazywanie wiadomości danego typu pomiędzy nodeami. **Najczęściej wykorzystywane są do komunikacji w jedną stronę, szczególnie jeśli chcemy żeby wiele nodeów słuchało wiadomości (np. podczas strumieni danych z sensorów).**

Na [Lab 1]() napisaliśmy już dwa nodey, jeden subskrybujący topic, a drugi publikujący na niego.

1. Dodajemy do paczki `basics` node `publisher.py`:

   ```py
   #!/usr/bin/env python3
   import rospy
   from std_msgs.msg import String

   rospy.init_node("publisher")
   publisher = rospy.Publisher("hello_world", String, queue_size=10)
   r = rospy.Rate(10)
   while not rospy.is_shutdown():
       publisher.publish("Hello World")
       r.sleep()
   ```

2. Dodajemy do paczki `basics` node `subscriber.py`:

   ```py
   #!/usr/bin/env python3
   import rospy
   from std_msgs.msg import String

   def callback(msg):
       print(msg)

   rospy.init_node("subscriber")
   subscriber = rospy.Subscriber("hello_world", String, callback)
   rospy.spin()
   ```

3. Pamiętamy o nadaniu uprawnienień:
   ```sh
   chmod +x publisher.py
   ```
   ```sh
   chmod +x subscriber.py
   ```
4. Uruchamiamy nodey w dwóch terminalach:

   ```sh
   rosrun basics publisher.py
   ```

   ```sh
   rusrun basics subscriber.py
   ```

5. Listę topiców zarejestrowanych w danym momencie przez ROSa sprawdzamy poleceniem:
   ```sh
   rostopic list
   ```
6. Aby dowiedzieć się więcej o jakimś topicu wpisujemy:

   ```sh
   rostopic info /scan
   ```

7. Aby słuchać wiadomości na topicu korzystamy z:

   ```sh
   rostopic echo /imu
   ```

   ```sh
   rostopic echo -n1 /imu # tylko jeden message
   ```

   ```sh
   rostopic echo /scan/header # tylko pole header
   ```

8. Aby publikować wiadomości na topic używamy:
   ```sh
   rostopic pub /hello_world std_msgs/String "data: 'hello world'"
   ```
9. Listę dostępnych wiadomości sprawdzić możemy poleceniem:

   ```sh
   rosmsg list
   ```

10. Aby dowiedzieć się o nich więcej korzystamy z:

    ```sh
    rosmsg info /LaserScan
    ```

**Uwaga:** Tworzenie własnych wiadmości nie zostało opisane na zajęciach, ale jest bardzo zbliżone do tworzenia serwisów. [Więcej informacji tutaj](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).

**Czytaj więcej:** [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv), [Writing a Simple Publisher and Subscriber Python](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).

## Services

<img src="https://docs.ros.org/en/foxy/_images/Service-SingleServiceClient.gif" width=500/>

**Serwisy najczęściej wykorzystywane są do prostych interakcji składających się z zapytania i odpowiedzi (np. zapytanie o aktualny stan nodea, poproszenie o zmianę stanu nodea).**

1. Tworzymy plik definiujący nowy serwis, umieścimy go w `catkin_ws/src/basics/srv` i nazwiemy `WordCount.srv`:
   ```py
   string words
   ---
   uint32 count
   ```
   Pierwsza część to request, a druga to resonse.
2. Modyfikujemy `CMakeLists.txt` w naszej paczce (`catkin_ws/src/basics/CMakeLists.txt`):
   1. ```cmake
      # powinno już być dodane na etapie
      # catkin_create_catkin_pkg ... std_msgs message_generation
      find_package(catkin REQUIRED COMPONENTS
        # ...
        std_msgs
        message_generation
      )
      ```
   2. ```cmake
      add_service_files(
        FILES
        WordCount.srv
        # ...
      )
      ```
   3. ```cmake
      generate_messages(
        DEPENDENCIES
        std_msgs
        # ...
      )
      ```
3. Powinniśmy też dodać:
   ```xml
    <build_depend>message_generation</build_depend>
   ```
   w odpowiednie miejsce `package.xml`, ale dzięki wywołanym wcześniej poleceniu
   ```sh
   catkin_create_catkin_pkg ... message_generation ...
   ```
   zostało to już dodane.
4. Budujemy paczkę:
   ```sh
   catkin_make
   ```
5. Sourceujemy workspace:
   ```sh
   source ./devel/setup.bash
   ```
6. Sprawdzamy dostępne definicja serwisów:
   ```sh
   rossrv list # wszystkie widoczne
   ```
   ```sh
   rossrv package basics # wszystkie widoczne w paczke
   ```
   ```sh
   rossrv info basics/WordList # info o serwisie
   ```
7. Tworzymy nowy node, który będzie serwerem serwisu:

   ```sh
   #!/usr/bin/env python3
   import rospy
   from basics.srv import WordCount, WordCountResponse


   def count_words(request):
       return WordCountResponse(len(request.words.split()))


   rospy.init_node("service_server")
   service = rospy.Service("word_count", WordCount, count_words)
   rospy.spin()
   ```

8. Tworzymy nowy node, który będzie klientem serwisu:

   ```sh
   #!/usr/bin/env python3
   import rospy
   from basics.srv import WordCount
   import sys

   rospy.init_node("service_client")
   rospy.wait_for_service("word_count")
   word_counter = rospy.ServiceProxy("word_count", WordCount)
   words = ' '.join(sys.argv[1:])
   word_count = word_counter(words)
   print(words, '->', word_count.count)
   ```

9. Pamiętamy o nadaniu uprawnień:
   ```py
   chmod +x service_client.py
   ```
   ```py
   chmod +x service_server.py
   ```
10. Korzystając z poleceń

- `rosrun ...` - uruchamia node
- `rosservice list` - wypisuje wszystkie działające serwisy
- `rosservice call ...` - wywołuje serwis (np. `rosservice call /word_count "words: 'bit ros'"`)

możemy testować nasze programy.

**Czytaj więcej:** [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv), [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).

## Actions

**Akcje wykorzystywane są do bardziej złożonych interakcji.** Mamy możliwość zadania goala, otrzymywania cyklicznego feedbacku, przerwania wykonywania goala. Po zakończeniu akcji otrzymujemy response. Wykonywanie akcji dzieli się na etapy, które odpowiadają swoim statusą (`actionlib_msgs/GoalStatusArray`).

Poniżej przygotujemy bardzo prosty serwer i klient akcji. Nie skorzystamy ze wszystkich dobrodziejstw tego interfejsu.

1. Tworzymy plik definiujący nową akcję, umieścimy go w `catkin_ws/src/basics/actions` i nazwiemy `Timer.action`:
   ```py
    duration time_to_wait
    ---
    duration time_elapsed
    uint32 updates_sent
    ---
    duration time_elapsed
    duration time_remaining
   ```
   Pierwsza część to goal, a druga to response, trzecia to cykliczny feedback.
2. Modyfikujemy `CMakeLists.txt` w naszej paczce (`catkin_ws/src/basics/CMakeLists.txt`):
   1. ```cmake
      # powinno już być dodane na etapie
      # catkin_create_catkin_pkg ... std_msgs message_generation
      find_package(catkin REQUIRED COMPONENTS
        # ...
        actionlib_msgs
        message_generation
      )
      ```
   2. ```cmake
      add_action_files(
        FILES
        Timer.action
        # ...
      )
      ```
   3. ```cmake
      generate_messages(
        DEPENDENCIES
        actionlib_msgs
        # ...
      )
      ```
3. Powinniśmy też dodać:
   ```xml
      <build_depend>actionlib_msgs</build_depend>
      <exec_depend>actionlib_msgs</exec_depend>
   ```
   w odpowiednie miejsce `package.xml`, ale dzięki wywołanym wcześniej poleceniu
   ```sh
   catkin_create_catkin_pkg ... actionlib_msgs ...
   ```
   zostało to już dodane.
4. Budujemy paczkę:
   ```sh
   catkin_make
   ```
5. Sourceujemy workspace:
   ```sh
   source ./devel/setup.bash
   ```
6. Tworzymy nowy node, który będzie serwerem akcji:

   ```py
   #!/usr/bin/env python3
   import rospy
   import time
   import actionlib
   from basics.msg import TimerAction, TimerGoal, TimerResult


   def do_timer(goal):
      start_time = time.time()
      rospy.sleep(goal.time_to_wait.to_sec())
      result = TimerResult()
      result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
      result.updates_sent = 0
      server.set_succeeded(result)


   rospy.init_node('timer_action_server')
   server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
   server.start()
   rospy.spin()
   ```

7. Tworzymy nowy node, który będzie klientem akcji:

   ```py
   #!/usr/bin/env python3
   import rospy
   import actionlib
   from basics.msg import TimerAction, TimerGoal, TimerResult

   rospy.init_node('timer_action_client')
   client = actionlib.SimpleActionClient('timer', TimerAction)
   client.wait_for_server()
   goal = TimerGoal()
   goal.time_to_wait = rospy.Duration.from_sec(5.0)
   client.send_goal(goal)
   client.wait_for_result()
   print('Time elapsed: %f' % (client.get_result().time_elapsed.to_sec()))
   ```

8. Korzystając z poleceń

   - `rosrun ...` - uruchamia node,
   - `rostopic list` - wypisuje wszystkie działające topici,
   - `rostopic info` - wypisuje informację o topicu,
   - `rostopic pub ...` - wywołuje serwis,

   możemy testować nasze programy.

   **Uwaga:** Akcje są zbudwane na topicach, więc dlatego korzystamy z `rostopic`.

**Czytaj więcej:** [actionlib](http://wiki.ros.org/actionlib)

## Źródła

- [Ładne diagramy](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS Wiki](http://wiki.ros.org/Documentation)
- [Programming Robots with ROS, O'Reilly Media](https://www.oreilly.com/library/view/programming-robots-with/9781449325480/) - książka dotyczy starszej wersji ROSa.
