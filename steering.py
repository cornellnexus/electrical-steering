from sshkeyboard import listen_keyboard, stop_listening

switch_vals = ["ON","OFF"]
switch = 0

def press(key):
  print(key)
  global switch, switch_vals
  if key in ["q","esc"]:
    stop_listening()
  if key == "space":
    switch += 1
    print(f"toggling switch {switch_vals[switch % 2]}")
  if key == "up":
    print(f"pressing up")

def release(key):
  print(key)
  if key == "up":
    print(f"up let go")

print("Listening to keybord input (over SSH). Press \"space\" to toggle, and \"ESC\" or \"q\" to quit")

listen_keyboard(
  on_press=press,
  on_release=release,
)