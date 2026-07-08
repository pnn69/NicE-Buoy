let socket =
new WebSocket(
"ws://" + location.hostname + ":81"
);

socket.onmessage = function(event)
{
    let d = JSON.parse(event.data);

    rawNeedle.style.transform =
        "rotate(" + d.raw + "deg)";

    calNeedle.style.transform =
        "rotate(" + d.cal + "deg)";

    tiltNeedle.style.transform =
        "rotate(" + d.tilt + "deg)";

    pitch.innerHTML = d.pitch;
    roll.innerHTML = d.roll;

    counter.innerHTML =
        d.count + " / 500";

    progress.value = d.count;

    if(d.running)
        calImage.classList.add("spin");
    else
        calImage.classList.remove("spin");
};

function startCal()
{
    fetch("/start");
}

function resetCal()
{
    fetch("/reset");
}