package main

import (
	"encoding/xml"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	stl "github.com/nfranczak/stl-bounding-box"
)

// URDF XML structures
type Robot struct {
	XMLName xml.Name `xml:"robot"`
	Name    string   `xml:"name,attr"`
	Links   []Link   `xml:"link"`
	Joints  []Joint  `xml:"joint"`
}

type Link struct {
	XMLName   xml.Name    `xml:"link"`
	Name      string      `xml:"name,attr"`
	Visual    []Visual    `xml:"visual"`
	Collision []Collision `xml:"collision"`
	Inertial  *Inertial   `xml:"inertial"`
	Origin    *Origin     `xml:"origin"`
}

type Visual struct {
	XMLName  xml.Name  `xml:"visual"`
	Origin   *Origin   `xml:"origin"`
	Geometry *Geometry `xml:"geometry"`
}

type Collision struct {
	XMLName  xml.Name  `xml:"collision"`
	Origin   *Origin   `xml:"origin"`
	Geometry *Geometry `xml:"geometry"`
}

type Inertial struct {
	XMLName xml.Name `xml:"inertial"`
	Mass    *Mass    `xml:"mass"`
	Origin  *Origin  `xml:"origin"`
	Inertia *Inertia `xml:"inertia"`
}

type Mass struct {
	XMLName xml.Name `xml:"mass"`
	Value   float64  `xml:"value,attr"`
}

type Origin struct {
	XMLName xml.Name `xml:"origin"`
	RPY     string   `xml:"rpy,attr,omitempty"`
	XYZ     string   `xml:"xyz,attr,omitempty"`
}

type Inertia struct {
	XMLName xml.Name `xml:"inertia"`
	IXX     float64  `xml:"ixx,attr"`
	IXY     float64  `xml:"ixy,attr"`
	IXZ     float64  `xml:"ixz,attr"`
	IYY     float64  `xml:"iyy,attr"`
	IYZ     float64  `xml:"iyz,attr"`
	IZZ     float64  `xml:"izz,attr"`
}

type Geometry struct {
	XMLName xml.Name `xml:"geometry"`
	Mesh    *Mesh    `xml:"mesh"`
	Box     *Box     `xml:"box"`
}

type Mesh struct {
	XMLName  xml.Name `xml:"mesh"`
	Filename string   `xml:"filename,attr"`
}

type Box struct {
	XMLName xml.Name `xml:"box"`
	Size    string   `xml:"size,attr"`
}

type Joint struct {
	XMLName  xml.Name  `xml:"joint"`
	Name     string    `xml:"name,attr"`
	Type     string    `xml:"type,attr"`
	Parent   *Parent   `xml:"parent"`
	Child    *Child    `xml:"child"`
	Origin   *Origin   `xml:"origin"`
	Axis     *Axis     `xml:"axis"`
	Limit    *Limit    `xml:"limit"`
	Dynamics *Dynamics `xml:"dynamics"`
}

type Parent struct {
	XMLName xml.Name `xml:"parent"`
	Link    string   `xml:"link,attr"`
}

type Child struct {
	XMLName xml.Name `xml:"child"`
	Link    string   `xml:"link,attr"`
}

type Axis struct {
	XMLName xml.Name `xml:"axis"`
	XYZ     string   `xml:"xyz,attr"`
}

type Limit struct {
	XMLName  xml.Name `xml:"limit"`
	Effort   float64  `xml:"effort,attr"`
	Lower    float64  `xml:"lower,attr"`
	Upper    float64  `xml:"upper,attr"`
	Velocity float64  `xml:"velocity,attr"`
}

type Dynamics struct {
	XMLName  xml.Name `xml:"dynamics"`
	Damping  float64  `xml:"damping,attr"`
	Friction float64  `xml:"friction,attr"`
}

func main() {
	if len(os.Args) < 3 {
		fmt.Println("Usage: urdf-simplifier <input.urdf> <output.urdf>")
		fmt.Println("  input.urdf  - Path to the input URDF file")
		fmt.Println("  output.urdf - Path to write the simplified URDF file")
		os.Exit(1)
	}

	inputPath := os.Args[1]
	outputPath := os.Args[2]

	// Read input URDF
	data, err := os.ReadFile(inputPath)
	if err != nil {
		fmt.Printf("Error reading input file: %v\n", err)
		os.Exit(1)
	}

	// Parse URDF
	var robot Robot
	if err := xml.Unmarshal(data, &robot); err != nil {
		fmt.Printf("Error parsing URDF: %v\n", err)
		os.Exit(1)
	}

	// Get base directory for resolving package:// URIs
	baseDir := filepath.Dir(inputPath)

	// Process links
	for i := range robot.Links {
		processLink(&robot.Links[i], baseDir)
	}

	// Filter to keep only the main kinematic chain
	filterToMainChain(&robot)

	// Marshal back to XML
	output, err := xml.MarshalIndent(robot, "", "  ")
	if err != nil {
		fmt.Printf("Error generating output XML: %v\n", err)
		os.Exit(1)
	}

	// Add XML header
	finalOutput := []byte(xml.Header + string(output) + "\n")

	// Write output
	if err := os.WriteFile(outputPath, finalOutput, 0644); err != nil {
		fmt.Printf("Error writing output file: %v\n", err)
		os.Exit(1)
	}

	fmt.Printf("Successfully simplified URDF: %s -> %s\n", inputPath, outputPath)
}

// filterToMainChain keeps only the main kinematic chain (revolute/prismatic joints)
// and removes all fixed joints and extra links like world, base, ft_frame, flange, tool0
func filterToMainChain(robot *Robot) {
	// Find all revolute and prismatic joints (the main kinematic chain)
	var mainJoints []Joint
	for _, joint := range robot.Joints {
		if joint.Type == "revolute" || joint.Type == "prismatic" {
			mainJoints = append(mainJoints, joint)
		}
	}

	// Build a set of link names that are part of the main chain
	linkSet := make(map[string]bool)
	for _, joint := range mainJoints {
		if joint.Parent != nil {
			linkSet[joint.Parent.Link] = true
		}
		if joint.Child != nil {
			linkSet[joint.Child.Link] = true
		}
	}

	// Filter links to keep only those in the main chain
	var filteredLinks []Link
	for _, link := range robot.Links {
		if linkSet[link.Name] {
			filteredLinks = append(filteredLinks, link)
		}
	}

	robot.Links = filteredLinks
	robot.Joints = mainJoints

	fmt.Printf("Filtered to main kinematic chain: %d links, %d joints\n", len(robot.Links), len(robot.Joints))
}

func processLink(link *Link, baseDir string) {
	// Step 1.3: Move origin from inertial to link level
	if link.Inertial != nil && link.Inertial.Origin != nil {
		link.Origin = link.Inertial.Origin
	}

	// Step 1.3: Remove inertial entirely
	link.Inertial = nil

	// Step 1.4: Remove visual elements
	link.Visual = nil

	// Step 2: Replace collision meshes with bounding boxes
	for i := range link.Collision {
		if link.Collision[i].Geometry != nil && link.Collision[i].Geometry.Mesh != nil {
			mesh := link.Collision[i].Geometry.Mesh

			// Resolve package:// URI to file path
			stlPath := resolvePackageURI(mesh.Filename, baseDir)
			fmt.Println("stlPath: ", stlPath)

			// Calculate bounding box
			bbox, err := stl.CalculateBoundingBoxFromFile(stlPath)

			if err != nil {
				fmt.Printf("Warning: Could not calculate bounding box for %s: %v\n", mesh.Filename, err)
				continue
			}

			// Get dimensions
			width, height, depth := bbox.Dimensions()

			// Get center coordinates
			center := bbox.Center

			// Replace mesh with box
			link.Collision[i].Geometry.Mesh = nil
			link.Collision[i].Geometry.Box = &Box{
				Size: fmt.Sprintf("%f %f %f", width, height, depth),
			}

			// Set or update the collision origin with the bounding box center
			if link.Collision[i].Origin == nil {
				link.Collision[i].Origin = &Origin{}
			}
			link.Collision[i].Origin.XYZ = fmt.Sprintf("%f %f %f", center.X, center.Y, center.Z)

			fmt.Printf("Replaced mesh %s with box of width, height, depth = (%.5f x %.5f x %.5f)\n",
				filepath.Base(stlPath), width, height, depth)
			fmt.Printf("Set collision origin to center: (%.5f, %.5f, %.5f)\n", center.X, center.Y, center.Z)
			fmt.Println(" ")
		}
	}
}

// resolvePackageURI resolves mesh file paths, handling both package:// URIs and regular paths
// Supports:
//   - package://ur_description/meshes/ur20/collision/shoulder.stl
//   - meshes/shoulder.stl (relative path)
//   - /absolute/path/to/shoulder.stl
func resolvePackageURI(uri string, baseDir string) string {
	// fmt.Println("uri: ", uri)
	// fmt.Println("baseDir: ", baseDir)
	// Handle package:// URIs
	if strings.HasPrefix(uri, "package://") {
		// Remove "package://" prefix
		relativePath := strings.TrimPrefix(uri, "package://")

		// Strip the package name (first component) from the path
		// e.g., "ur_description/meshes/ur20/collision/base.stl" -> "meshes/ur20/collision/base.stl"
		parts := strings.SplitN(relativePath, "/", 2)
		if len(parts) == 2 {
			relativePath = parts[1]
		}

		standardPath := filepath.Join(baseDir, relativePath)
		// fmt.Println("relativePath: ", relativePath)
		// fmt.Println("standardPath: ", standardPath)

		// Check if standard path exists
		if _, err := os.Stat(standardPath); err == nil {
			return standardPath
		}

		// If not found, search for a file matching the relative path suffix
		var foundPath string
		filepath.Walk(baseDir, func(path string, info os.FileInfo, err error) error {
			if err != nil {
				return nil
			}
			if !info.IsDir() && strings.HasSuffix(path, relativePath) {
				// fmt.Println("path: ", path)
				foundPath = path
				// fmt.Println("RETURNING HERE NOW")
				return filepath.SkipAll
			}
			return nil
		})
		// fmt.Println("foundPath: ", foundPath)

		if foundPath != "" {
			return foundPath
		}

		// Return standard path even if it doesn't exist (will fail later with clear error)
		return standardPath
	}

	// Handle absolute paths - use as-is
	if filepath.IsAbs(uri) {
		return uri
	}

	// Handle relative paths - resolve relative to baseDir
	return filepath.Join(baseDir, uri)
}
