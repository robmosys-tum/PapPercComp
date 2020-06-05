## Using the knowledge base

This implementation makes sense within the cakin workspace, see the full KnowRob repository [here](https://github.com/jkabalar/knowrob) with the JsonPrologWrapper added.

### Instructions (json_prolog)

From the catkin workspace, perform

```bash
roslaunch json_prolog json_prolog.launch
```
to launch the interface. 

Test the parsing using one of the example clients, for example by executing

```bash
./examples/test_json_prolog.py
```
to test the python implementation. Or try

```bash
rosrun json_prolog test_json_prolog
```
to test the cpp client that is able to communicate to ROS2 using the ros1bridge.

---

Output of the test

```bash
./examples/test_json_prolog.py
[INFO] [1591103273.253279]: waiting for json_prolog services
[INFO] [1591103273.263179]: json_prolog services ready
Found solution. A = 1, B = [u'x', 1]
Found solution. A = 2, B = [u'x', 2]
Found solution. A = 3, B = [u'x', 3]
Found solution. A = 4, B = [u'x', 4]
```


