# Running tests


Make sure to build the project first and have a rosmaster running. You can call rostest directly with:

```
rostest rll_planning_project tests_python.test
```

If you want to see the test execution in RVIZ you can pass launch options as usual, as well as rostest options.

```
rostest rll_planning_project tests_python.test --text headless:=false
```


