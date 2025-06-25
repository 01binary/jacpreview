# JacPreview

A web-based real-time Jacobian matrix visualization tool using ROS 2 Bridge and Next.js.

## Description

JacPreview is a web application that receives Jacobian matrix data from ROS 2 topics via ROS 2 Bridge and visualizes them in real-time using React and Next.js. Perfect for robotics applications requiring live matrix analysis and visualization.

## Features

- Real-time Jacobian matrix visualization from ROS 2 topics
- Web-based interface using Next.js and React
- ROS 2 Bridge integration for message handling
- Interactive matrix analysis tools
- Responsive design for various screen sizes
- Live data streaming and visualization

## Getting Started

### Prerequisites

- Node.js 18+ and npm
- ROS 2 (Humble or later)
- ros2bridge package
- Modern web browser

### Installation

1. Clone the repository:
```bash
git clone https://github.com/01binary/jacpreview.git
cd jacpreview
```

2. Install dependencies:
```bash
npm install
```

3. Set up ROS 2 Bridge (if not already installed):
```bash
# Install ros2bridge
sudo apt install ros-humble-ros2bridge
```

4. Run the development server:
```bash
npm run dev
```

5. Open [http://localhost:3000](http://localhost:3000) in your browser

### ROS 2 Setup

1. Start ROS 2 Bridge:
```bash
ros2 launch ros2bridge ros2bridge_launch.py
```

2. Publish Jacobian matrix data to the configured topic (default: `/jacobian_matrix`)

## Usage

1. Start the web application using `npm run dev`
2. Ensure ROS 2 Bridge is running and connected
3. Publish Jacobian matrix messages to the configured ROS 2 topic
4. View real-time visualization in the web interface

## Project Structure

```
jacpreview/
├── src/
│   ├── components/     # React components
│   ├── pages/         # Next.js pages
│   ├── lib/           # Utility functions
│   └── styles/        # CSS styles
├── public/            # Static assets
├── package.json       # Dependencies
└── next.config.js     # Next.js configuration
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built with Next.js and React for modern web development
- Integrated with ROS 2 Bridge for real-time robotics data
- Designed for live Jacobian matrix visualization and analysis 