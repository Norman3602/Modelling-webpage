document.querySelectorAll(".nav-toggle").forEach((button) => {
  button.addEventListener("click", () => {
    const menu = document.getElementById(button.getAttribute("aria-controls"));
    const isOpen = !menu.hidden;

    document.querySelectorAll(".dropdown").forEach((d) => (d.hidden = true));
    document
      .querySelectorAll(".nav-toggle")
      .forEach((b) => b.setAttribute("aria-expanded", "false"));

    menu.hidden = isOpen;
    button.setAttribute("aria-expanded", String(!isOpen));
  });
});

document.addEventListener("click", (e) => {
  if (!e.target.closest(".nav-group")) {
    document.querySelectorAll(".dropdown").forEach((d) => (d.hidden = true));
    document
      .querySelectorAll(".nav-toggle")
      .forEach((b) => b.setAttribute("aria-expanded", "false"));
  }
});

const botPopup = document.getElementById("botPopup");
const botText = document.getElementById("botText");

function showBot(message) {
  botText.textContent = message;
  botPopup.hidden = false;
}

let botShown = false;

window.addEventListener("scroll", () => {
  if (window.scrollY > 350 && !botShown) {
    botShown = true;
    showBot(
      "Need some guidance? Start with Control Fundamentals and explore the learning activities!",
    );
  }
});

document.getElementById("askBot").addEventListener("click", () => {
  showBot(
    "Hello! I'm ControlBot. Start with Steady-State Error, then explore PID Control and the drone simulation!",
  );
});

document.getElementById("closeBot").addEventListener("click", () => {
  botPopup.hidden = true;
});
