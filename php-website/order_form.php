<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

</head>

<body>

<?php
$soda = $can = $water = $chip = "0";
$err = "";

if ($_SERVER["REQUEST_METHOD"] == "POST") {

	$soda = intval($_POST["soda"]);
	$can = intval($_POST["can"]);
	$water = intval($_POST["water"]);
	$chip = intval($_POST["chip"]);
	
	$total = $soda + $can + $water + $chip;

	if ($total == 0) {
		$err = "No selection";
	} elseif ($total > 2) {
		$err = "Too many items! Maximum of 2";
	} else {
		$myfile = fopen("/var/www/html/new/orders.txt", "a+") or die("Unable to open file!");
		$txt = "Order: " . $soda . "," . $can . "," . $water . "," . $chip . "\n";
		fwrite($myfile, $txt);
		fclose($myfile);
		header("Location: order.php");

	}
}

?>

  <h1>Order</h1>
  <p>Select your items. Only 2 items are allowed</p>
  
  <form method="post" action="<?php echo htmlspecialchars($_SERVER["PHP_SELF"]);?>">
	Soda (Bottle):
	<input type="radio" name="soda" value="0" checked="checked">0
	<input type="radio" name="soda" value="1">1
	<input type="radio" name="soda" value="2">2
	<br><br>
	Soda (Cans):
	<input type="radio" name="can" value="0" checked="checked">0
	<input type="radio" name="can" value="1">1
	<input type="radio" name="can" value="2">2
	<br><br>
	Water (Bottle):
	<input type="radio" name="water" value="0" checked="checked">0
	<input type="radio" name="water" value="1">1
	<input type="radio" name="water" value="2">2
	<br><br>
	Chips:
	<input type="radio" name="chip" value="0" checked="checked">0
	<input type="radio" name="chip" value="1">1
	<input type="radio" name="chip" value="2">2
	<br><br>
	<input type="submit" name="submit" value="Submit">
</form>

<span class="error"><?php echo $err;?></span>

</body>
</html>
