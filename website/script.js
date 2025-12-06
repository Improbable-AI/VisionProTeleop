/**
 * VisionProTeleop Landing Page - Interactive Scripts
 */

document.addEventListener('DOMContentLoaded', () => {
    // Smooth scroll for anchor links
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            e.preventDefault();
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                const navHeight = document.querySelector('.nav').offsetHeight;
                const targetPosition = target.getBoundingClientRect().top + window.pageYOffset - navHeight - 20;
                window.scrollTo({
                    top: targetPosition,
                    behavior: 'smooth'
                });
            }
        });
    });

    // Animate elements on scroll
    const observerOptions = {
        root: null,
        rootMargin: '0px',
        threshold: 0.1
    };

    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('animate-in');
                observer.unobserve(entry.target);
            }
        });
    }, observerOptions);

    // Observe feature cards and component cards
    document.querySelectorAll('.feature-card, .component-card, .step').forEach(el => {
        el.style.opacity = '0';
        el.style.transform = 'translateY(20px)';
        el.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
        observer.observe(el);
    });

    // Add animate-in class styles
    const style = document.createElement('style');
    style.textContent = `
        .animate-in {
            opacity: 1 !important;
            transform: translateY(0) !important;
        }
    `;
    document.head.appendChild(style);

    // Parallax effect for hero visual
    const heroVisual = document.querySelector('.hero-visual');
    if (heroVisual) {
        window.addEventListener('scroll', () => {
            const scrolled = window.pageYOffset;
            const rate = scrolled * 0.3;
            heroVisual.style.transform = `translateY(${rate}px)`;
        });
    }

    // Nav background on scroll
    const nav = document.querySelector('.nav');
    let lastScroll = 0;

    window.addEventListener('scroll', () => {
        const currentScroll = window.pageYOffset;

        if (currentScroll > 50) {
            nav.style.background = 'rgba(10, 10, 11, 0.95)';
        } else {
            nav.style.background = 'rgba(10, 10, 11, 0.8)';
        }

        lastScroll = currentScroll;
    });

    // Add stagger animation to grid items
    const staggerElements = (elements, baseDelay = 100) => {
        elements.forEach((el, index) => {
            el.style.transitionDelay = `${index * baseDelay}ms`;
        });
    };

    staggerElements(document.querySelectorAll('.feature-card'), 100);
    staggerElements(document.querySelectorAll('.component-card'), 150);
    staggerElements(document.querySelectorAll('.step'), 150);

    // Typing Animation
    const typingPhrases = [
        'Teleoperate Real World Robots',
        'Teleoperate in Simulation',
        'Record Egocentric Human Videos',
        'Stream Hand/Head Tracking',
        'Stream Low-Latency Video/Audio',
        'Render MuJoCo in AR',
        'Log Robot Datasets to Cloud'
    ];

    const typingElement = document.querySelector('.typing-text');
    const cursorElement = document.querySelector('.typing-cursor');

    if (typingElement) {
        let phraseIndex = 0;
        let charIndex = 0;
        let isDeleting = false;
        let isPaused = false;

        const typeSpeed = 50;      // ms per character when typing
        const deleteSpeed = 30;    // ms per character when deleting
        const pauseBeforeDelete = 2000;  // pause after typing complete
        const pauseBeforeType = 500;     // pause after deleting complete

        function typeAnimation() {
            const currentPhrase = typingPhrases[phraseIndex];

            if (isPaused) {
                return;
            }

            if (!isDeleting) {
                // Typing
                const text = currentPhrase.substring(0, charIndex + 1);
                typingElement.innerHTML = text || '&nbsp;';
                charIndex++;

                if (charIndex === currentPhrase.length) {
                    // Finished typing, pause then start deleting
                    isPaused = true;
                    setTimeout(() => {
                        isPaused = false;
                        isDeleting = true;
                        typeAnimation();
                    }, pauseBeforeDelete);
                    return;
                }

                setTimeout(typeAnimation, typeSpeed);
            } else {
                // Deleting
                const text = currentPhrase.substring(0, charIndex - 1);
                typingElement.innerHTML = text || '&nbsp;';
                charIndex--;

                if (charIndex === 0) {
                    // Finished deleting, move to next phrase
                    isDeleting = false;
                    phraseIndex = (phraseIndex + 1) % typingPhrases.length;
                    isPaused = true;
                    setTimeout(() => {
                        isPaused = false;
                        typeAnimation();
                    }, pauseBeforeType);
                    return;
                }

                setTimeout(typeAnimation, deleteSpeed);
            }
        }

        // Start the animation
        setTimeout(typeAnimation, 500);
    }
});
