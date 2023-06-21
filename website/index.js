
updateProgressBar(-50);

function updateProgressBar(value) {
    console.log(value);
    // Get the fill element
    const fill = document.querySelector('.progress-bar-fill');
    // Update the progress bar
    // Calculate the percentage
    const percentage = Math.abs(value) * 100 / 100;
    // Calculate the height of the fill element
    const fillHeight = percentage * 2;
    // Set the height of the fill element
    fill.style.height = fillHeight + 'px';
    // Apply class based on value
    if (value < 0) {
        fill.classList.remove('positive');
        fill.classList.add('negative');
    } else if (value > 0) {
        fill.classList.remove('negative');
        fill.classList.add('positive');
    } else {
        fill.classList.remove('negative');
        fill.classList.remove('positive');
    }
}
