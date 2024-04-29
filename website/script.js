document.getElementById('inputValue').addEventListener('input', function() {
    var value = parseInt(this.value);
    var bar = document.getElementById('bar');
    
    if (value < 0) {
      bar.style.width = Math.abs(value) + 'px';
      bar.className = 'bar red'; // Add 'bar' class to maintain initial styling
    } else {
      bar.style.width = value + 'px';
      bar.className = 'bar green'; // Add 'bar' class to maintain initial styling
    }
  });
  