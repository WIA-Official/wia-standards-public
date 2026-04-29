<?
//보내는 이
  //$recipient = "dreamuto@naver.com";
  $recipient = "wia@wia.family";
//제목 처리
 $subject = '=?UTF-8?B?'.base64_encode( "[WBCIA] - 문의 메일입니다." ).'?=';    
//메일주소
  $mail_from = '=?UTF-8?B?'.base64_encode($_GET['email']).'?=';

//메일내용
  $mail_body = "<table width='600' border='0' cellpadding='0' cellspacing='1' bgcolor='#CCCCCC'>
        <tr> 
          <td width='100' height='30' align='center' bgcolor='#eeeeee'>이름</td>
          <td width='400' bgcolor='#FFFFFF'>". $_GET['userName']."</td></tr>".

       " <tr> 
          <td width='100' height='30' align='center' bgcolor='#eeeeee'>이메일 주소</td>
          <td width='400' bgcolor='#FFFFFF'>". $_GET['email']."</td></tr>".		
		  
       " <tr> 
          <td width='100' height='30' align='center' bgcolor='#eeeeee'>연락처</td>
          <td width='400' bgcolor='#FFFFFF'>". $_GET['phone']."</td></tr>".	
		  
       " <tr> 
          <td width='100' height='30' align='center' bgcolor='#eeeeee'>내용</td>
          <td width='400' bgcolor='#FFFFFF'>". nl2br($_GET['content'])."</td></tr>".	
		  
      "</table>";

 
//메일 발송처리

  $header = "Content-Type: text/html;charset=UTF-8\n";
  $header .= "From: $mail_from <".$mail_from.">\n"; 

 
  $email = mail($recipient, $subject, $mail_body, $header);



  if (!$email)
    echo "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" /> <script>
         window.alert('Failed to send mail!');
         history.go(-1);
         </script>";
  else
    echo "         <script>
         window.alert('Suceess to send mail!');
         history.go(-1);
         </script>";
?>
