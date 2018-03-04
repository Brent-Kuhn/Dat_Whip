$(function() {
  $("#form").submit(function(event) {
    event.preventDefault();
  });
  $("input[type=button]").click(function(event) {
    var $updateButton = $(event.currentTarget);
    var $row = $updateButton.parents().eq(1);
    var key = keyFromRow($row);
    var value = valueFromRow($row);
    postUpdate(key, value);
  });

  function keyFromRow($row) {
    return $row.children().eq(0).text().trim();
  }

  function valueFromRow($row) {
    return $row.find("input[type=text]").val().trim();
  }

  function textFromChild($elem, index) {
    return $elem
  }

  function postUpdate(key, value) {
    var data = {};
    data[key] = value;
    $.post("/update", data, function(data, status, jqXHR) {
      if (status != 200) {
        console.error("Failed to update value (" + key + ", " + value + ").");
      }
    });
  }
});
