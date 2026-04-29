const mail = {
  setData: function () {
    var formArr = $("form").serializeArray();
    var obj = {
      name: formArr[0].value,
      phone: formArr[2].value,
      to: formArr[1].value,
      contents: formArr[3].value,
    };
    return obj;
  },
  send: function () {
    $.ajax({
      url: "http://wbcia.io/mail/send",
      type: "POST",
      data: JSON.stringify(mail.setData()),
      contentType: "application/json",
      beforeSend: function () {
        var bh = $("body")[0].scrollHeight;
        $("body").css("height", bh);
        $("body").append(
          '<div id="mask" style="height:' +
            bh +
            'px"></div><span id="mask_text">문의 접수 중입니다.</span>'
        );
      },
      success: function (r) {
        $("#mask").remove();
        $("#mask_text").remove();
        if (r == "success") {
          alert("접수되었습니다.");
        } else {
          alert(
            "오류가 발생했습니다. 지속적으로 발생시 관리자에게 문의주세요."
          );
        }
      },
    });
  },
};

$(function () {
  $("#Form").validate({
    rules: {
      userName: {
        required: true,
      },
      email: {
        email: true,
        required: true,
      },
      phone: {
        required: true,
      },
      content: {
        required: true,
      },
    },
    messages: {
      userName: {
        required: "필수 입력입니다.",
      },
      email: {
        required: "필수 입력입니다.",
        email: "이메일 형식을 확인해주세요.",
      },
      phone: {
        required: "필수 입력입니다.",
      },
      content: {
        required: true,
      },
    },
    submitHandler: function (form) {
      mail.send();
    },
  });
});
