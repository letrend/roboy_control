FILE(REMOVE_RECURSE
  "CMakeFiles/roboy_control_generate_messages_py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/_Trajectory.py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/_InitializeRequest.py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/_Steer.py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/_Status.py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/_InitializeResponse.py"
  "../devel/lib/python2.7/dist-packages/roboy_control/msg/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/roboy_control_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
