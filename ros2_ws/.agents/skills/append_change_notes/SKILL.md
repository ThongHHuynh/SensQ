---
name: append_change_notes
description: Triggers ONLY when the user adds the [NOTE] tag in their prompt. Instructs the agent to append a concise note about the specific problem, subject, and resolution to a changelog file.
---

# Instructions

When the user includes the `[NOTE]` tag in their request, you MUST document the change by appending a note to `CHANGELOG.md` located at the root of the workspace (`/home/tom/SensQ/ros2_ws/CHANGELOG.md`). Do NOT append notes unless the `[NOTE]` tag is explicitly present.

For each documented change, follow these strict guidelines:
1. Include the current date and time.
2. Explicitly state the related **Subject** (e.g., navigation, mapping, control, etc.).
3. Explicitly state the specific **Problem** that was fixed or the feature that was added.
4. Explicitly state the **Resolution** or the changes made.
5. Keep the note extremely concise.

### Format Template
```markdown
## [YYYY-MM-DD]
* **Subject**: <related subject, e.g., mapping, navigation, etc.>
* **Problem**: <concise description of the specific problem>
* **Resolution**: <concise description of how it was resolved>
```

Do not include any extra pleasantries or verbose explanations in the changelog. Only document actual changes made to the codebase.
