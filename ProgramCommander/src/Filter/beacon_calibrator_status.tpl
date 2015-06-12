<div class="ComponentPanel" id="{{component_name}}">
  <div class="MessagesContainer" id="{{component_name}}_messages">
    <div class="PanelHeader" id="{{component_name}}_header">{{component_name}}</div>
    <div class="Message" id="{{component_name}}_status">{{status}}</div>
    <div class="Time" id="{{component_name}}_up_time">{{up_time}}</div>
    <div class="ProcessId" id="{{component_name}}_process_id">{{process_id}}</div>
    <div class="StatusString" id="{{component_name}}_status_str">{{status_str}}</div>
  </div>
  <div class="ButtonsContainer" id="{{component_name}}_buttons">
    <div class="SelectionColumn">
      <div class="Selection White" id="{{component_name}}_model">None</div>
      <div class="Button Blue" id="{{component_name}}_cycle">Model</div>
      <div class="Button Blue" id="{{component_name}}_runas">Live</div>      
      <div class="Button Yellow" id="{{component_name}}_select">Select</div>
      <div class="Button White" id="{{component_name}}_b4"></div>      
    </div>
    <div class="ButtonsColumn">
      <div class="StartButton" id="{{component_name}}_start">START</div>
      <div class="Button Yellow" id="{{component_name}}_log">Log</div>
      <div class="ExitButton" id="{{component_name}}_exit">EXIT</div>      
    </div>
  </div>
</div>
<script>
  var model_list_{{component_name}} = [
    "None",
    "Camera00",
    "Camera01",
    "Pixy00",
    "Pixy01"
  ];
  var runas_list_{{component_name}} = [
    "Live",
    "Hist"
  ];
  var model_num_{{component_name}} = 0;
  var runas_num_{{component_name}} = 0;
  var selection_state_{{component_name}} = 0;  // 0 = select, 1 = are you sure?
  $("div#{{component_name}}_cycle").click(function() {
    if (selection_state_{{component_name}} == 0) {
      model_num_{{component_name}}++;
      if (model_num_{{component_name}} >= model_list_{{component_name}}.length) {
        model_num_{{component_name}} = 0;
      }
      $("div#{{component_name}}_model").text(
        model_list_{{component_name}}[model_num_{{component_name}}]
      );
    } else {
      send_to_server({msg: "COMMAND {{component_name}} model "+
        model_list_{{component_name}}[model_num_{{component_name}}]+" "+
        runas_list_{{component_name}}[runas_num_{{component_name}}]
      });
      selection_state_{{component_name}} = 0;
      $("div#{{component_name}}_cycle").text("Model");
      $("div#{{component_name}}_select").text("Select");
    }
  });
  $("div#{{component_name}}_select").click(function() {
    if (selection_state_{{component_name}} == 0) {
      selection_state_{{component_name}} = 1;
      $("div#{{component_name}}_cycle").text("Yes");
      $("div#{{component_name}}_select").text("No");
    } else {
      selection_state_{{component_name}} = 0;
      $("div#{{component_name}}_cycle").text("Model");
      $("div#{{component_name}}_select").text("Select");
    }
  });
  $("div#{{component_name}}_runas").click(function() {
    runas_num_{{component_name}}++;
    if (runas_num_{{component_name}} >= runas_list_{{component_name}}.length) {
      runas_num_{{component_name}} = 0;
    }
    $("div#{{component_name}}_runas").text(
      runas_list_{{component_name}}[runas_num_{{component_name}}]
    );
  });
  function update_{{component_name}}(data) {
    $("div#{{component_name}}_status").text(data.status);
    $("div#{{component_name}}_up_time").text(data.up_time);
    $("div#{{component_name}}_process_id").text(data.process_id);
    $("div#{{component_name}}_status_str").text(data.status_str);
  }
  $("div#{{component_name}}_start").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} start"});
  });
  $("div#{{component_name}}_log").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} log"});
  });
  $("div#{{component_name}}_exit").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} exit"});
  });
  $("div#{{component_name}}_play").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} play"});
  });
  $("div#{{component_name}}_stop").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} stop"});
  });
  $("div#{{component_name}}_pause").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} pause"});
  });
</script>
