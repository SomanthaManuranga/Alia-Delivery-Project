function createParticles() {
    const particlesContainer = document.querySelector(".particles");
    const particleCount = 25;

    for (let i = 0; i < particleCount; i++) {
        const particle = document.createElement("div");
        particle.className = "particle";
        particle.style.left = Math.random() * 100 + "%";
        particle.style.top = Math.random() * 100 + "%";
        particle.style.animationDelay = Math.random() * 5 + "s";
        particle.style.animationDuration = 3 + Math.random() * 4 + "s";
        particlesContainer.appendChild(particle);
    }
}

// Initialize particles
createParticles();

let selectedRoom = "101"; // Default selected room

// send button click listener
document.addEventListener("click", function (e) {
    if (e.target.closest(".room-card")) {
        const card = e.target.closest(".room-card");
        const room = card.dataset.room;

        // Remove previous selection
        document
            .querySelectorAll(".room-card")
            .forEach((c) => c.classList.remove("selected"));

        // Add selection to clicked room card
        card.classList.add("selected");
        selectedRoom = room;
    }
});

function sendRoomCommand() {
    const room = selectedRoom;
    const statusEl = document.getElementById("status");
    const button = document.querySelector(".send-button");

    // Update button style and message status for loading state
    button.style.opacity = "0.7";
    button.style.pointerEvents = "none";
    statusEl.innerHTML =
        '<div class="loading-spinner"></div>Sending robot to room ' +
        room +
        "...";
    statusEl.className = "status-loading";

    fetch("/send", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ room: room }),
    })
        .then((res) => res.json())
        .then((data) => {
            statusEl.innerHTML = "✅ " + data.status + ": " + data.command;
            statusEl.className = "status-success";
        })
        .catch((error) => {
            setTimeout(() => {
                statusEl.className = "status-failure";
            }, 1200);
        })
        .finally(() => {
            // Reset button state
            setTimeout(() => {
                button.style.opacity = "1";
                button.style.pointerEvents = "auto";
            }, 1200);
        });
}
